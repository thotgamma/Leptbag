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

	elementNode[string] parts;
	generic6DofConstraint[string] g6dofs;
	serialOrderGene SOG;
	//oscillator2Gene gene; //振動子モデル+DEにおける遺伝子
	agentBodyParameter bodyInformation;
	int sequenceOfOrder; //SOG.tracksのsequenceOfOrder番目の命令を動作に用いる
	int biologicalClock; //現在のsequenceOfOrderになってからどのくらい時間が経ったか

	this(float x, float y, float z, agentBodyParameter info){


		this.bodyInformation = info;
		foreach(string name, params;bodyInformation.partParams){
			writeln("\t", name);
			this.bodyInformation.partsGenerator[name] = new elementManager(this.bodyInformation.partParams[name].vertices, &createConvexHullShapeBody);
		}
		this.spawn(Vector3f(x, y, z));


		this.SOG.init();
		//this.gene.init();

		foreach(string s, dof; g6dofs){
			if(this.bodyInformation.g6dofParams[s].enabled){
				this.SOG.init(s, this.bodyInformation.g6dofParams[s].angLimitLower, this.bodyInformation.g6dofParams[s].angLimitUpper);
			}
			//this.gene.init(s);
		}

		//this.gene.rehash();

	}


	void spawn(Vector3f position){

		this.sequenceOfOrder = 0;
		this.biologicalClock = 0;

		//身体パーツ
		foreach(string s, partsGen; bodyInformation.partsGenerator){

			parts[s] = partsGen.generate(
						parameterPack(
							param("position", (bodyInformation.partParams[s].position + position) ),
							param("scale",    bodyInformation.partParams[s].scale),
							param("rotation", bodyInformation.partParams[s].rotation),
							param("model",    bodyInformation.partParams[s].vertices),
							param("mass",bodyInformation.partParams[s].mass * bodyMass)
						)
						);

		}

		//6Dof
		foreach(string s, param; bodyInformation.g6dofParams){
			if(this.bodyInformation.g6dofParams[s].enabled){
				g6dofs[s] = new generic6DofConstraint(parts[bodyInformation.g6dofParams[s].object1Name], parts[bodyInformation.g6dofParams[s].object2Name],
						bodyInformation.g6dofParams[s].object1Position, bodyInformation.g6dofParams[s].object2Position,
						bodyInformation.g6dofParams[s].rotation);
			}
		}

		parts = parts.rehash;
		g6dofs = g6dofs.rehash;



		foreach(part; parts) part.setFriction(this.SOG.friction);


		foreach(string s, dof; g6dofs){

			if(this.bodyInformation.g6dofParams[s].enabled){

				for(int i=0; i<3; i++){
					if(bodyInformation.g6dofParams[s].useAngLimit[i]) g6dofs[s].setRotationalMotor(i);
					if(bodyInformation.g6dofParams[s].useLinLimit[i]) g6dofs[s].setLinearMotor(i);
				}


				//for test
				Vector3f testVector3fLow = Vector3f( -1.57/2.0, -1.57/2.0, -1.57/2.0 );
				Vector3f testVector3fUp = Vector3f( 1.57/2.0, 1.57/2.0, 1.57/2.0 );

				Vector3f zeroVector3f = Vector3f( 0.0, 0.0, 0.0 ); //セッターに同じVector3fを入れるとロック


				//setting angular of g6dofs
				bool useAng = true;
				for(int i=0; i<bodyInformation.g6dofParams[s].useAngLimit.length; i++) useAng = useAng && bodyInformation.g6dofParams[s].useAngLimit[i];
				if(useAng){
					g6dofs[s].setAngularLimit(
							bodyInformation.g6dofParams[s].angLimitLower,
							bodyInformation.g6dofParams[s].angLimitUpper );
				}else{
					g6dofs[s].setAngularLimit( zeroVector3f, zeroVector3f );
				}


				//setting linear of g6dofs
				bool useLin = true;
				for(int i=0; i<bodyInformation.g6dofParams[s].useLinLimit.length; i++) useLin = useLin && bodyInformation.g6dofParams[s].useLinLimit[i];
				if(false){//useLin){
					g6dofs[s].setLinearLimit( bodyInformation.g6dofParams[s].linLimitLower, bodyInformation.g6dofParams[s].linLimitUpper );
				}else{
					g6dofs[s].setLinearLimit( zeroVector3f, zeroVector3f );
				}

				//最大出力．index ; (x, y, z)=(0, 1, 2)(たぶん)
				g6dofs[s].setMaxRotationalMotorForce( 0, this.SOG.maxRotationalMotorForce);
				g6dofs[s].setMaxRotationalMotorForce( 1, this.SOG.maxRotationalMotorForce);
				g6dofs[s].setMaxRotationalMotorForce( 2, this.SOG.maxRotationalMotorForce);

				}

			}


		}

		void despawn(){
			foreach(string s, part; parts){
				part.destroy();
		}
		foreach(string s, dofs; g6dofs){
			if(this.bodyInformation.g6dofParams[s].enabled){
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
		if(++this.biologicalClock==this.SOG.moveSpan[this.sequenceOfOrder]){
			this.sequenceOfOrder = (++this.sequenceOfOrder)%serialOrderGene.lengthOfSet;
			this.biologicalClock = 0;
		}
	}


	void moveWithSerialOrder(){

		if(g6dofs.length==0) return;

		/*
		writeln("wavelengthOfOrder.length:", this.SOG.wavelengthOfOrder.length, "\nsequenceOfOrder:", sequenceOfOrder, "\nbiologicalClock:", biologicalClock, "\nmoveSpan.length:", this.SOG.moveSpan.length);
		writeln("moveSpan:", this.SOG.moveSpan);
		writeln("wavelengthOfOrder:", this.SOG.wavelengthOfOrder);
		*/

		if(!this.SOG.wavelengthOfOrder[sequenceOfOrder][biologicalClock]) return;

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


	/+
	void moveWithOsci(){

		//oscilは現在角度から次の駆動力を決定する
		if(g6dofs.length!=0) foreach(string s, dof; g6dofs){
			gene.oscil.setTheta(s, dof.getAngle(0)); //thetaがx方向の関節移動
			gene.oscil.setPhi(s, dof.getAngle(1)); //phiがz方向の関節移動
		}

		//theta, phiは角度ではなく，単に駆動力の度合いと考えてよい
		float[string] deltaTheta = gene.oscil.calculateDeltaTheta();
		float[string] deltaPhi = gene.oscil.calculateDeltaPhi();

		//sinでdeltaTheta, deltaPhiを-1.0~1.0にリミットしている
		foreach(string s, dof; g6dofs) dof.setRotationalTargetVelocity( Vector3f(
					gene.maxelo[s].getx()*sin(deltaTheta[s]), gene.maxVelo[s].gety()*sin(deltaPhi[s]), 0.0f ) );

	}
	+/





}
