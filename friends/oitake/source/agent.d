import core.runtime;
import std.stdio;
import std.math;
import std.json;
import std.random;
import std.conv;
import std.algorithm;

import japariSDK.japarilib;
import dlib.math.vector;
import dlib.math.quaternion;

//import DEforOscillator2;
//import Oscillator;
import params;
import loadJson;

Random rnd;


const float bodyMass = 10.0f; //動物の総体重．blender側では各パーツに百分率で質量を付与．


/*
elementManager[string] partsGenerator;


//fpmからの読取用
partParam[string] partParams; //身体パーツのパラメータ
g6dofParam[string] g6dofParams; //g6dofのパラメータ
*/



class agent{

	static agentBodyParameter bodyInformation;
	Vector3f initialPos;
	elementNode[string] parts;
	generic6DofConstraint[string] g6dofs;
	serialOrderGene SOG;
	int sequenceOfOrder; //SOG.tracksのsequenceOfOrder番目の命令を動作に用いる
	int biologicalClock; //現在のsequenceOfOrderになってからどのくらい時間が経ったか



	this(float x, float y, float z){

		this.SOG.init();
		this.spawn(Vector3f(x, y, z));


		foreach(string s, dof; g6dofs){
			if(agent.bodyInformation.g6dofParams[s].enabled){
				this.SOG.init(s, agent.bodyInformation.g6dofParams[s].angLimitLower, agent.bodyInformation.g6dofParams[s].angLimitUpper);
			}
		}


	}


	static void registerParameter(agentBodyParameter info);

	void spawn(Vector3f position){

		this.initialPos = position;

		this.sequenceOfOrder = 0;
		this.biologicalClock = 0;

		//身体パーツ
		foreach(string s, partsGen; agent.bodyInformation.partsGenerator){

			parts[s] = partsGen.generate(
						parameterPack(
							param("position", (agent.bodyInformation.partParams[s].position + position) ),
							param("scale",    agent.bodyInformation.partParams[s].scale),
							param("rotation", agent.bodyInformation.partParams[s].rotation),
							param("model",    agent.bodyInformation.partParams[s].vertices),
							param("mass",agent.bodyInformation.partParams[s].mass * bodyMass)
						)
						);

		}

		foreach(part; parts) part.setFriction(this.SOG.friction);

		//6Dof
		foreach(string s, param; agent.bodyInformation.g6dofParams){
			if(agent.bodyInformation.g6dofParams[s].enabled){
				g6dofs[s] = new generic6DofConstraint(parts[agent.bodyInformation.g6dofParams[s].object1Name], parts[agent.bodyInformation.g6dofParams[s].object2Name],
						agent.bodyInformation.g6dofParams[s].object1Position, agent.bodyInformation.g6dofParams[s].object2Position,
						agent.bodyInformation.g6dofParams[s].rotation);

				for(int i=0; i<3; i++){
					if(agent.bodyInformation.g6dofParams[s].useAngLimit[i]) g6dofs[s].setRotationalMotor(i);
					if(agent.bodyInformation.g6dofParams[s].useLinLimit[i]) g6dofs[s].setLinearMotor(i);
				}

				Vector3f zeroVector3f = Vector3f( 0.0, 0.0, 0.0 ); //セッターに同じVector3fを入れるとロック

				//setting angular of g6dofs
				bool useAng = true;
				for(int i=0; i<agent.bodyInformation.g6dofParams[s].useAngLimit.length; i++) useAng = useAng && agent.bodyInformation.g6dofParams[s].useAngLimit[i];
				if(useAng){
					g6dofs[s].setAngularLimit(
							agent.bodyInformation.g6dofParams[s].angLimitLower,
							agent.bodyInformation.g6dofParams[s].angLimitUpper );
				}else{
					g6dofs[s].setAngularLimit( zeroVector3f, zeroVector3f );
				}

				//最大出力．index ; (x, y, z)=(0, 1, 2)(たぶん)
				g6dofs[s].setMaxRotationalMotorForce( 0, this.SOG.maxRotationalMotorForce);
				g6dofs[s].setMaxRotationalMotorForce( 1, this.SOG.maxRotationalMotorForce);
				g6dofs[s].setMaxRotationalMotorForce( 2, this.SOG.maxRotationalMotorForce);

				//setting linear of g6dofs
				bool useLin = true;
				for(int i=0; i<agent.bodyInformation.g6dofParams[s].useLinLimit.length; i++) useLin = useLin && agent.bodyInformation.g6dofParams[s].useLinLimit[i];
				if(false){//useLin){
					g6dofs[s].setLinearLimit( agent.bodyInformation.g6dofParams[s].linLimitLower, agent.bodyInformation.g6dofParams[s].linLimitUpper );
				}else{
					g6dofs[s].setLinearLimit( zeroVector3f, zeroVector3f );
				}

				g6dofs[s].setMaxLinearMotorForce(Vector3f(0.0f, 0.0f, 0.0f));



			}
		}



			parts = parts.rehash;
			g6dofs = g6dofs.rehash;

		}

		void despawn(){
			foreach(string s, part; parts){
				part.destroy();
		}
		foreach(string s, dofs; g6dofs){
			if(agent.bodyInformation.g6dofParams[s].enabled){
				dofs.destroy();
			}
		}
	}



	void copyGene(agent parent){
		//this.gene = parent.gene;
		this.SOG.tracks = parent.SOG.tracks;
	}


	void checkSOG(){

		writeln("check SOG");
		foreach(string s, dof; g6dofs){
			if(s=="Constraint"){
				writeln(s);
				for(int i=0; i<SOG.tracks.length; i++){
					write("\t", i, " : (");
					writeln(SOG.tracks[i][s]);
				}
			}
		}

	}


	void updateBiologicalClock(){
		//if(++this.biologicalClock==this.SOG.moveSpan[this.sequenceOfOrder]){
		if(++this.biologicalClock==6){
			this.sequenceOfOrder = (++this.sequenceOfOrder)%serialOrderGene.lengthOfSet;
			this.biologicalClock = 0;
		}
		//}
	}


	void moveWithSerialOrder(){

		if(g6dofs.length==0) return;


		foreach(string s, dof; g6dofs){

			Vector3f currentAngle = Vector3f(dof.getAngle(0), dof.getAngle(1), dof.getAngle(2));

			dof.setRotationalTargetVelocity(
					Vector3f(
						this.SOG.maxVelocity[sequenceOfOrder][s].x
						*(SOG.tracks[sequenceOfOrder][s].x - currentAngle.x),
						this.SOG.maxVelocity[sequenceOfOrder][s].y
						*(SOG.tracks[sequenceOfOrder][s].y - currentAngle.y),
						this.SOG.maxVelocity[sequenceOfOrder][s].z
						*(SOG.tracks[sequenceOfOrder][s].z - currentAngle.z)
						)
					);
		}

	}

	}


	static void registerParameter(agentBodyParameter info){

		agent.bodyInformation = info;
		foreach(string name, params;agent.bodyInformation.partParams){
			agent.bodyInformation.partsGenerator[name] = new elementManager(agent.bodyInformation.partParams[name].vertices, &createConvexHullShapeBody);
		}

	}

