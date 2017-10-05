import core.runtime;
import std.stdio;
import std.math;
import std.json;
import std.random;
import std.conv;
import std.algorithm;

import japariSDK.japarilib;
import DEforOscillator2;
import Oscillator;
import params;
import loadJson;

Random rnd;


const float bodyMass = 10.0f; //動物の総体重．blender側では各パーツに百分率で質量を付与．


/*
elementManager[string] partsGenerator;


//fpmからの読取用
partParam[string] partParams; //身体パーツのパラメータ
hingeParam[string] hingeParams; //ヒンジのパラメータ
g6dofParam[string] g6dofParams; //g6dofのパラメータ
*/



class agent{

	elementNode[string] parts;
	hingeConstraint[string] hinges;
	generic6DofConstraint[string] g6dofs;
	serialOrderGene SOG;
	oscillator2Gene gene; //振動子モデル+DEにおける遺伝子
	agentBodyParameter bodyInformation;


	this(float x, float y, float z, agentBodyParameter info){

		this.bodyInformation = info;
		foreach(string name, params;bodyInformation.partParams){
			bodyInformation.partsGenerator[name] = createElementManager(bodyInformation.partParams[name].vertices, &createConvexHullShapeBody);
		}
		spawn(createVec3(x, y, z));
		SOG.init();
		gene.init();
		foreach(string s, dof; g6dofs){
			SOG.init(s, this.bodyInformation.g6dofParams[s].angLimitLower, this.bodyInformation.g6dofParams[s].angLimitUpper);
			gene.init(s);
		}
		gene.rehash();

	}

	void spawn(vec3 position){


		//身体パーツ
		foreach(string s, partsGen; bodyInformation.partsGenerator){

			parts[s] = partsGen.generate(paramWrap(
						param("position", addVec(bodyInformation.partParams[s].position, position)),
						param("scale",    bodyInformation.partParams[s].scale),
						param("rotation", bodyInformation.partParams[s].rotation),
						param("model",    bodyInformation.partParams[s].vertices),
						param("mass",
						bodyInformation.partParams[s].mass * bodyMass)));
						//0.0f)));

		}


		//ヒンジ
		foreach(string s, param; bodyInformation.hingeParams){
			hinges[s] = hingeConstraint_create(parts[bodyInformation.hingeParams[s].object1Name], parts[bodyInformation.hingeParams[s].object2Name],
					bodyInformation.hingeParams[s].object1Position, bodyInformation.hingeParams[s].object2Position,
					bodyInformation.hingeParams[s].axis1, bodyInformation.hingeParams[s].axis2);
			hinges[s].setLimit( bodyInformation.hingeParams[s].limitLower, bodyInformation.hingeParams[s].limitLower );
			if( bodyInformation.hingeParams[s].enabled ){
				hinges[s].enableMotor(true);
				hinges[s].setMaxMotorImpulse(5);
			}
		}

		//6Dof
		foreach(string s, param; bodyInformation.g6dofParams){
			g6dofs[s] = generic6DofConstraint_create(parts[bodyInformation.g6dofParams[s].object1Name], parts[bodyInformation.g6dofParams[s].object2Name],
					bodyInformation.g6dofParams[s].object1Position, bodyInformation.g6dofParams[s].object2Position,
					bodyInformation.g6dofParams[s].rotation);
		}

		parts = parts.rehash;
		hinges = hinges.rehash;
		g6dofs = g6dofs.rehash;



		foreach(part; parts) part.setFriction(3.5);


		foreach(string s, dof; g6dofs){



			for(int i=0; i<3; i++){
				if(bodyInformation.g6dofParams[s].useAngLimit[i]) g6dofs[s].setRotationalMotor(i);
				if(bodyInformation.g6dofParams[s].useLinLimit[i]) g6dofs[s].setLinearMotor(i);
			}


			//for test
			vec3 testVec3Low = createVec3( -1.57/2.0, -1.57/2.0, -1.57/2.0 );
			vec3 testVec3Up = createVec3( 1.57/2.0, 1.57/2.0, 1.57/2.0 );

			vec3 zeroVec3 = createVec3( 0.0, 0.0, 0.0 ); //セッターに同じvec3を入れるとロック

			bool useAng = true;
			for(int i=0; i<bodyInformation.g6dofParams[s].useAngLimit.length; i++) useAng = useAng && bodyInformation.g6dofParams[s].useAngLimit[i];
			if(useAng){
				g6dofs[s].setAngularLimit(
						bodyInformation.g6dofParams[s].angLimitLower,
						bodyInformation.g6dofParams[s].angLimitUpper );
			}else{
				g6dofs[s].setAngularLimit( zeroVec3, zeroVec3 );
			}


			bool useLin = true;
			for(int i=0; i<bodyInformation.g6dofParams[s].useLinLimit.length; i++) useLin = useLin && bodyInformation.g6dofParams[s].useLinLimit[i];
			if(false){//useLin){
				g6dofs[s].setLinearLimit( bodyInformation.g6dofParams[s].linLimitLower, bodyInformation.g6dofParams[s].linLimitUpper );
			}else{
				g6dofs[s].setLinearLimit( zeroVec3, zeroVec3 );
			}

			//最大出力．index ; (x, y, z)=(0, 1, 2)(たぶん？)
			g6dofs[s].setMaxRotationalMotorForce( 0, 10.0); //gene.maxForce[s].getx() );
			g6dofs[s].setMaxRotationalMotorForce( 1, 10.0); //gene.maxForce[s].gety() );
			g6dofs[s].setMaxRotationalMotorForce( 2, 10.0); //gene.maxForce[s].getz() );


		}


	}

	void despawn(){
		foreach(string s, part; parts){
			part.destroy();
		}
		foreach(hinge; hinges) hinge.destroy();
		foreach(dofs; g6dofs) dofs.destroy();
	}

	void moveWithSerialOrder(int sequence){

		if(g6dofs.length!=0)
			foreach(string s, dof; g6dofs){
				dof.setRotationalTargetVelocity( createVec3(
					5.0*(SOG.tracks[sequence][s].getx() - dof.getAngle(0)),
					0.0f, //10.0*(SOG.tracks[sequence][s].gety() - dof.getAngle(1)),
					0.0f
					)
				);
			}

	}


	void copyGene(agent parent){
		this.gene = parent.gene;
		this.SOG.copytracks(parent.SOG);
	}


	void checkSOG(){

		writeln("check SOG");
		foreach(string s, dof; g6dofs){
			if(s=="Constraint"){
				writeln(s);
				for(int i=0; i<SOG.lengthOfSet; i++){
					write("\t", i, " : (");
					write(SOG.tracks[i][s].getx(), ", ");
					write(SOG.tracks[i][s].gety(), ", ");
					writeln(SOG.tracks[i][s].getz(), " )");
				}
			}
		}

	}



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
		foreach(string s, dof; g6dofs) dof.setRotationalTargetVelocity( createVec3(
					gene.maxVelo[s].getx()*sin(deltaTheta[s]), gene.maxVelo[s].gety()*sin(deltaPhi[s]), 0.0f ) );

	}





}
