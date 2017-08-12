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

const int agentNum = 100;
const int moveSpan = 12;

string measuredPart = "head"; //この名前のパーツの移動距離を測る
const float bodyMass = 5.0f; //動物の総体重．blender側では各パーツに百分率で質量を付与．

agent[] agents; //メイン
agent[] evaluateds; //DEにおける突然変異個体

elementManager[string] partsGenerator;


//fpmからの読取用
partParam[string] partParams; //身体パーツのパラメータ
hingeParam[string] hingeParams; //ヒンジのパラメータ
g6dofParam[string] g6dofParams; //g6dofのパラメータ



class agent{

	elementNode[string] parts;
	hingeConstraint[string] hinges;
	generic6DofConstraint[string] g6dofs;
	oscillator2Gene gene; //振動子モデル+DEにおける遺伝子


	this(float x, float y, float z){

		spawn(createVec3(x, y, z));

	}


	void spawn(vec3 position){

		foreach(string name, params;partParams){
				partsGenerator[name] = createElementManager(partParams[name].vertices, &createConvexHullShapeBody);
		}


		foreach(string s, elementManager partsGen; partsGenerator){

			parts[s] = partsGen.generate(paramWrap(
						param("position", addVec(partParams[s].position, position)),
						param("scale",    partParams[s].scale),
						param("rotation", partParams[s].rotation),
						param("model",    partParams[s].vertices),
						param("mass",
							//0.0f)));
				partParams[s].mass * bodyMass)));

		}

		//ヒンジ
		foreach(string s, param; hingeParams){
			hinges[s] = hingeConstraint_create(parts[hingeParams[s].object1Name], parts[hingeParams[s].object2Name],
					hingeParams[s].object1Position, hingeParams[s].object2Position,
					hingeParams[s].axis1, hingeParams[s].axis2);
			hinges[s].setLimit( hingeParams[s].limitLower, hingeParams[s].limitLower );
			if( hingeParams[s].enabled ){
				hinges[s].enableMotor(true);
				hinges[s].setMaxMotorImpulse(5);
			}
		}

		//6Dof
		foreach(string s, param; g6dofParams){
			g6dofs[s] = generic6DofConstraint_create(parts[g6dofParams[s].object1Name], parts[g6dofParams[s].object2Name],
					g6dofParams[s].object1Position, g6dofParams[s].object2Position,
					g6dofParams[s].rotation);
		}

		parts = parts.rehash;
		hinges = hinges.rehash;
		g6dofs = g6dofs.rehash;

		gene.init();
		foreach(part; parts) part.setFriction(gene.friction);

		foreach(string s, dof; g6dofs){

			gene.init(s);


			for(int i=0; i<3; i++){
				if(g6dofParams[s].useAngLimit[i]) g6dofs[s].setRotationalMotor(i);
				if(g6dofParams[s].useLinLimit[i]) g6dofs[s].setLinearMotor(i);
			}


			//for test
			vec3 testVec3Low = createVec3( -1.57/2.0, -1.57/2.0, -1.57/2.0 );
			vec3 testVec3Up = createVec3( 1.57/2.0, 1.57/2.0, 1.57/2.0 );

			vec3 zeroVec3 = createVec3( 0.0, 0.0, 0.0 ); //セッターに同じvec3を入れるとロック

			g6dofs[s].setAngularLimit( gene.angLimitLower[s], gene.angLimitUpper[s] );
			g6dofs[s].setLinearLimit( zeroVec3, zeroVec3 );

			//最大出力．index ; (x, y, z)=(0, 1, 2)(たぶん？)
			g6dofs[s].setMaxRotationalMotorForce( 0, gene.maxForce[s].getx() );
			g6dofs[s].setMaxRotationalMotorForce( 1, gene.maxForce[s].gety() );
			g6dofs[s].setMaxRotationalMotorForce( 2, gene.maxForce[s].getz() );


		}
		gene.rehash();


	}

	void move(){

		if(g6dofs.length!=0) foreach(string s, dof; g6dofs){
			gene.oscil.setTheta(s, dof.getAngle(0)); //thetaがx方向の関節移動
			gene.oscil.setPhi(s, dof.getAngle(1)); //phiがz方向の関節移動
		}

		//theta, phiは角度ではなく，単に移動を行う度合いと考えてよい
		float[string] deltaTheta = gene.oscil.calculateDeltaTheta();
		float[string] deltaPhi = gene.oscil.calculateDeltaPhi();

		//sinでdeltaTheta, deltaPhiを-1.0~1.0にリミットしている
		foreach(string s, dof; g6dofs) dof.setRotationalTargetVelocity( createVec3(
					gene.maxVelo[s].getx()*sin(deltaTheta[s]), gene.maxVelo[s].gety()*sin(deltaPhi[s]), 0.0f ) );


		/+
			if(hinges.length!=0) foreach(string s, hinge; hinges){
				if(hingeParams[s].enabled){
					float target = abs(hingeParams[s].limitLower-hingeParams[s].limitUpper) * dna[sequence][s] * 2.0/PI;
					hinge.setMotorTarget(target, 0.5);
				}
			}
		+/



	}

	void despawn(){
		foreach(part; parts) part.destroy();
		foreach(hinge; hinges) hinge.destroy();
		foreach(dofs; g6dofs) dofs.destroy();
	}


}


//ApplicationInterface----------------------

extern (C) void init(){
	try{
		rt_init();
		Random(unpredictableSeed);
		writeln("lowPolyFox_6dof_DE.d loaded");


		loadMesh(partParams);
		loadHinge(hingeParams);
		loadG6dof(g6dofParams);

		partParams = partParams.rehash;
		hingeParams = hingeParams.rehash;
		g6dofParams = g6dofParams.rehash;



		//agents生成
		agents.length = agentNum;
		foreach(int i, ref elem; agents) elem = new agent(to!float(i)*5.0f, 0.0f, -1.0f);

		//最初が0世代目
		writeln("start generation : 0");



	}
	catch (Exception ex){
		writeln(ex.toString);
	}



}



//マイステップ実行されるべき動作----------------------------

bool evaluation = false; //trueならDEの突然変異体評価フェイズ
float topRecord = 0.0f; //動物たちは-z方向に歩いていることに注意
float averageRecord = 0.0f; //毎世代ごとの平均到達距離表示用
int timerDivisor = 0;
int time = 0; //時計
int generation = 0; //世代を記録する
float[agentNum] preRecords; //前回の移動距離を保存しておき，突然変異によってより大きく移動すれば突然変異体を採用
float[agentNum] evaluatedsRecords;

int averageOf = 5; //一世代5回の試行を行いその平均をスコアとする
int trial = 0;
float score = 0.0f;

extern (C) void tick(){


	if(timerDivisor++ == moveSpan){
		timerDivisor = 0;
		if(!evaluation) foreach(elem; agents) elem.move();
		else foreach(elem; evaluateds) elem.move();
	}
	time++;


	//世代終わり
	if(time == (200 + generation*5)){


		float proRecordTmp = 0.0; //この世代の最高移動距離


		if(trial==averageOf){
			if(!evaluation){
				//評価フェイズ
				writeln("	start evaluation : ", generation);
			}else{
				writeln("start generation : ", ++generation);
			}
			trial = 0;
		}else{
			writeln("trial at : ", trial);
		}




		time = 0;
		trial++;


		if(!evaluation){ //各の移動距離を測るフェイズ

			//geneにはtoString()が(中途半端に)実装されている
			//agents[0].gene.toString();

			foreach(int i, ref elem; agents){

				//移動距離を記録
				preRecords[i] += elem.parts[measuredPart].getZpos();
				averageRecord += elem.parts[measuredPart].getZpos();

				//今回の最高記録
				if(proRecordTmp>elem.parts[measuredPart].getZpos()){
					proRecordTmp = elem.parts[measuredPart].getZpos();
				}

				elem.despawn();
				elem.spawn(createVec3(to!float(i)*5.0f, 0.0f, 0.0f));

			}

			//trial回の試行を終えたのでフェイズ移行
			if(trial==averageOf){

				//最良3個体を記録
				int[] bests = [ 0, 0, 0 ];
				//最良スコア
				float topScore = 0.0f;
				foreach(int i, ref elem ;agents){

					//agentは一旦退場
					elem.despawn();

					if(preRecords[i]<topScore){
						bests[2] = bests[1];
						bests[1] = bests[0];
						bests[0] = i;
					}

				}


				if(generation==0){ //最初に評価用の犬たちevaluatedsをつくる
					evaluateds.length = agentNum;
					foreach(int i, ref elem; evaluateds) elem = new agent(to!float(i)*5.0f, 0.0f, -1.0f);
				}else{ //0世代以降は突然変異を行う

					//DEに用いるパラメータ
					float ditherF = uniform(0.5f, 1.0f, rnd);
					//突然変異
					evolveBest(evaluateds, agents, 0.9f, ditherF, bests);

					//evaluatedsをpop
					foreach(int i, ref elem; evaluateds) elem.spawn(createVec3(to!float(i)*5.0f, 0.0f, 0.0f));

				}
				evaluation = true; //次は突然変異体評価フェイズ

			}

		}else{ //突然変異体評価フェイズ

			//evaluateds[0].gene.toString();

			foreach(int i, ref elem; evaluateds){

				evaluatedsRecords[i] += elem.parts[measuredPart].getZpos();
				averageRecord += elem.parts[measuredPart].getZpos();
				//今回の最高記録
				if(evaluateds[i].parts[measuredPart].getZpos() < proRecordTmp){
					proRecordTmp = evaluateds[i].parts[measuredPart].getZpos();
				}

				elem.despawn();
				elem.spawn(createVec3(to!float(i)*5.0f, 0.0f, 0.0f));

			}

			if(trial==averageOf){

				foreach(int i, ref elem; evaluateds){
					//もし突然変異した各個体が前回の同じindexの個体より良い性能なら採用
					if(evaluatedsRecords[i] <= preRecords[i]){
						agents[i].gene = elem.gene;
					}
					preRecords[i] = 0.0f;
					evaluatedsRecords[i] = 0.0f;

				}
				//突然変異体は一旦退場
				foreach(int i, ref elem; evaluateds) elem.despawn();
				foreach(int i, ref elem; agents) elem.spawn(createVec3(to!float(i)*5.0f, 0.0f, 0.0f));
				evaluation = false; //次は採用した突然変異体を混ぜて性能評価

			}

		}

		//最高記録を表示
		if(proRecordTmp<topRecord){
			topRecord = proRecordTmp;
			writeln("\n		new record! : ", -1.0*topRecord);
		}

		//今世代の最高記録
		writeln("		top achievement : ", -1.0f*proRecordTmp);

		//今世代の平均移動距離を表示
		writeln("		agerage achievement : ", -1.0f*averageRecord/to!float(agentNum));
		averageRecord = 0.0f;






	}




}



//------------------------------------------
