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


const float bodyMass = 5.0f; //動物の総体重．blender側では各パーツに百分率で質量を付与．


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
	agentBodyParameter bodyInfomation;


	this(float x, float y, float z, agentBodyParameter info){

		this.bodyInfomation = info;
		foreach(string name, params;bodyInfomation.partParams){
			bodyInfomation.partsGenerator[name] = createElementManager(bodyInfomation.partParams[name].vertices, &createConvexHullShapeBody);
		}
		spawn(createVec3(x, y, z));

	}

	void spawn(vec3 position){


		//身体パーツ
		foreach(string s, partsGen; bodyInfomation.partsGenerator){

			parts[s] = partsGen.generate(paramWrap(
						param("position", addVec(bodyInfomation.partParams[s].position, position)),
						param("scale",    bodyInfomation.partParams[s].scale),
						param("rotation", bodyInfomation.partParams[s].rotation),
						param("model",    bodyInfomation.partParams[s].vertices),
						param("mass",
						bodyInfomation.partParams[s].mass * bodyMass)));
						//0.0f)));

		}


		//ヒンジ
		foreach(string s, param; bodyInfomation.hingeParams){
			hinges[s] = hingeConstraint_create(parts[bodyInfomation.hingeParams[s].object1Name], parts[bodyInfomation.hingeParams[s].object2Name],
					bodyInfomation.hingeParams[s].object1Position, bodyInfomation.hingeParams[s].object2Position,
					bodyInfomation.hingeParams[s].axis1, bodyInfomation.hingeParams[s].axis2);
			hinges[s].setLimit( bodyInfomation.hingeParams[s].limitLower, bodyInfomation.hingeParams[s].limitLower );
			if( bodyInfomation.hingeParams[s].enabled ){
				hinges[s].enableMotor(true);
				hinges[s].setMaxMotorImpulse(5);
			}
		}

		//6Dof
		foreach(string s, param; bodyInfomation.g6dofParams){
			g6dofs[s] = generic6DofConstraint_create(parts[bodyInfomation.g6dofParams[s].object1Name], parts[bodyInfomation.g6dofParams[s].object2Name],
					bodyInfomation.g6dofParams[s].object1Position, bodyInfomation.g6dofParams[s].object2Position,
					bodyInfomation.g6dofParams[s].rotation);
		}

		parts = parts.rehash;
		hinges = hinges.rehash;
		g6dofs = g6dofs.rehash;


		SOG.init();
		gene.init();
		foreach(part; parts) part.setFriction(gene.friction);


		foreach(string s, dof; g6dofs){

			SOG.init(s);
			gene.init(s);

			for(int i=0; i<3; i++){
				if(bodyInfomation.g6dofParams[s].useAngLimit[i]) g6dofs[s].setRotationalMotor(i);
				if(bodyInfomation.g6dofParams[s].useLinLimit[i]) g6dofs[s].setLinearMotor(i);
			}


			//for test
			vec3 testVec3Low = createVec3( -1.57/2.0, -1.57/2.0, -1.57/2.0 );
			vec3 testVec3Up = createVec3( 1.57/2.0, 1.57/2.0, 1.57/2.0 );

			vec3 zeroVec3 = createVec3( 0.0, 0.0, 0.0 ); //セッターに同じvec3を入れるとロック

			g6dofs[s].setAngularLimit( bodyInfomation.g6dofParams[s].angLimitLower, bodyInfomation.g6dofParams[s].angLimitUpper );
			g6dofs[s].setLinearLimit( zeroVec3, zeroVec3 );

			//最大出力．index ; (x, y, z)=(0, 1, 2)(たぶん？)
			g6dofs[s].setMaxRotationalMotorForce( 0, 10.0); //gene.maxForce[s].getx() );
			g6dofs[s].setMaxRotationalMotorForce( 1, 10.0); //gene.maxForce[s].gety() );
			g6dofs[s].setMaxRotationalMotorForce( 2, 10.0); //gene.maxForce[s].getz() );


		}
		gene.rehash();


	}


	void moveWithSerialOrder(int sequence){

		if(g6dofs.length!=0)
			foreach(string s, dof; g6dofs){
				dof.setRotationalTargetVelocity( createVec3(
					4.0*(SOG.tracks[sequence][s].getx() - dof.getAngle(0)),
					0.0f, //10.0*(SOG.tracks[sequence][s].gety() - dof.getAngle(1)),
					0.0f
					)
				);
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


	void despawn(){
		foreach(part; parts) part.destroy();
		foreach(hinge; hinges) hinge.destroy();
		foreach(dofs; g6dofs) dofs.destroy();
	}


}
