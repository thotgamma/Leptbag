import core.runtime;
import std.stdio;
import std.math;
import std.json;
import std.random;
import std.conv;
import std.algorithm;

import japariSDK.japarilib;
import GA;
import DEforOscillator2;
import Oscillator;
import params;

Random rnd;

//strategy 0:do not learn, 1:HingeDE, 2:simple GA, 3:6dofDE
const int strategy = 3;
const int dogNum = 50;

string measuredPart = "head"; //この名前のパーツの移動距離を測る
const int attackProbability = 30; //simpleGAの突然変異確率
const float bodyMass = 42.592; //動物の総体重．blender側では各パーツに百分率で質量を付与．

chorodog[] chorodogs; //メイン
chorodog[] evaluateds; //DEにおける突然変異個体

elementManager[string] partsGenerator;


//fpmからの読取用
partParam[string] partParams; //身体パーツのパラメータ
hingeParam[string] hingeParams; //ヒンジのパラメータ
g6dofParam[string] g6dofParams; //g6dofのパラメータ




class chorodog{

	elementNode[string] parts;
	hingeConstraint[string] hinges;
	generic6DofConstraint[string] g6dofs;
	oscillator2Gene gene; //振動子モデル+DEにおける遺伝子

	float[string][20] dna; //simpleGAにおける遺伝子

	this(float x, float y, float z, bool initialDNA){


		spawn(createVec3(x, y, z));

		if(initialDNA == true){
			/*
			   foreach(string s, hinge; hinges){
			   if(hingeParams[s].enabled){
			   for(int row = 0; row < 20; row++){
			   dna[row][s] = uniform(-PI/2, PI/2, rnd);
			   }
			   }
			   }
			 */


		}

	}

	void spawn(vec3 position){


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

	void move(int sequence){

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
		writeln("chorodog.d loaded");



		//HACK コンパイル時にjsonStringにlowPolyTree.fpmの内容が代入される(要-Jオプション)
		//auto jsonString = import("models/chorodog.fpm");
		auto jsonString = import("models/chorodog6dof_simplified.fpm");
		//auto jsonString = import("models/lowPolyFox_trimed.fpm");
		//auto jsonString = import("models/chorodog_simplified.fpm");
		//auto jsonString = import("models/lowPolyFox_6dof.fpm");

		JSONValue model = parseJSON(jsonString);

		foreach(elem; model.array){
			if(elem["objectType"].str == "MESH"){

				string name = elem["name"].str;

				partParams[name] = partParam();
				partParams[name].position = createVec3(elem["xpos"].floating, elem["ypos"].floating, elem["zpos"].floating);
				partParams[name].scale = createVec3(elem["xscl"].floating, elem["yscl"].floating, elem["zscl"].floating);
				partParams[name].rotation = createQuat(elem["wqat"].floating, elem["xqat"].floating, elem["yqat"].floating, elem["zqat"].floating);
				partParams[name].mass = elem["mass"].floating;
				partParams[name].friction = elem["friction"].floating;

				partParams[name].vertices = createVertexManager();

				foreach(objvertex; elem["vertex"].array){
					partParams[name].vertices.addVertex(createVertex(objvertex.array[0].floating, objvertex.array[1].floating, objvertex.array[2].floating,
								objvertex.array[3].floating, objvertex.array[4].floating, objvertex.array[5].floating,
								objvertex.array[6].floating, objvertex.array[7].floating, objvertex.array[8].floating));
				}

				partsGenerator[name] = createElementManager(partParams[name].vertices, &createConvexHullShapeBody);

			}
		}

		foreach(elem; model.array){
			if(elem["objectType"].str == "hingeConstraint"){

				string name = elem["name"].str;
				hingeParams[name] = hingeParam();
				hingeParams[name].name = name;

				hingeParams[name].axis1 = createVec3(elem["xaxs1"].floating , elem["yaxs1"].floating, elem["zaxs1"].floating);
				hingeParams[name].axis2 = createVec3(elem["xaxs2"].floating , elem["yaxs2"].floating, elem["zaxs2"].floating);


				if(elem["enabled"].str == "True") hingeParams[name].enabled = true; else hingeParams[name].enabled = false;
				if(elem["useLimit"].str == "True") hingeParams[name].useLimit = true; else hingeParams[name].useLimit = false;
				hingeParams[name].limitLower = elem["limitLower"].floating;
				hingeParams[name].limitUpper = elem["limitUpper"].floating;
				hingeParams[name].object1Position = createVec3(elem["object1xpos"].floating, elem["object1ypos"].floating, elem["object1zpos"].floating);
				hingeParams[name].object2Position = createVec3(elem["object2xpos"].floating, elem["object2ypos"].floating, elem["object2zpos"].floating);
				hingeParams[name].object1Name = elem["object1"].str;
				hingeParams[name].object2Name = elem["object2"].str;


			}else if(elem["objectType"].str == "genericConstraint"){

				string name = elem["name"].str;
				g6dofParams[name] = g6dofParam();

				g6dofParams[name].name = name;
				if(elem["enabled"].str == "True") g6dofParams[name].enabled = true; else g6dofParams[name].enabled = false;
				g6dofParams[name].position = createVec3(to!float(elem["xpos"].str), to!float(elem["ypos"].str), to!float(elem["zpos"].str));
				g6dofParams[name].rotation = createQuat(elem["wqat"].floating, elem["xqat"].floating, elem["yqat"].floating, elem["zqat"].floating);
				g6dofParams[name].object1Name = elem["object1"].str;
				g6dofParams[name].object2Name = elem["object2"].str;
				g6dofParams[name].object1Position = createVec3(elem["object1xpos"].floating, elem["object1ypos"].floating, elem["object1zpos"].floating);
				g6dofParams[name].object2Position = createVec3(elem["object2xpos"].floating, elem["object2ypos"].floating, elem["object2zpos"].floating);
				if(elem["useXAngLimit"].str == "True") g6dofParams[name].useAngLimit[0]= true; else g6dofParams[name].useAngLimit[0] = false;
				if(elem["useYAngLimit"].str == "True") g6dofParams[name].useAngLimit[1]= true; else g6dofParams[name].useAngLimit[1] = false;
				if(elem["useZAngLimit"].str == "True") g6dofParams[name].useAngLimit[2]= true; else g6dofParams[name].useAngLimit[2] = false;

				g6dofParams[name].angLimitLower = createVec3(elem["xAngLimitLower"].floating, elem["yAngLimitLower"].floating, elem["zAngLimitLower"].floating);
				g6dofParams[name].angLimitUpper = createVec3(elem["xAngLimitUpper"].floating, elem["yAngLimitUpper"].floating, elem["zAngLimitUpper"].floating);

				if(elem["useXLinLimit"].str == "True") g6dofParams[name].useLinLimit[0]= true; else g6dofParams[name].useLinLimit[0] = false;
				if(elem["useYAngLimit"].str == "True") g6dofParams[name].useLinLimit[1]= true; else g6dofParams[name].useLinLimit[1] = false;
				if(elem["useZAngLimit"].str == "True") g6dofParams[name].useLinLimit[2]= true; else g6dofParams[name].useLinLimit[2] = false;

				g6dofParams[name].linLimitLower = createVec3(elem["xLinLimitLower"].floating, elem["yLinLimitLower"].floating, elem["zLinLimitLower"].floating);
				g6dofParams[name].linLimitUpper = createVec3(elem["xLinLimitUpper"].floating, elem["yLinLimitUpper"].floating, elem["zLinLimitUpper"].floating);


			}
		}
		partParams = partParams.rehash;
		hingeParams = hingeParams.rehash;
		g6dofParams = g6dofParams.rehash;



		//chorodogs生成
		chorodogs.length = dogNum;
		foreach(int i, ref elem; chorodogs) elem = new chorodog(to!float(i)*5.0f, 0.0f, -1.0f, true);

		//最初が0世代目
		if(strategy!=0) writeln("start generation : 0");



	}
	catch (Exception ex){
		writeln(ex.toString);
	}



}




bool evaluation = false; //trueならDEの突然変異体評価フェイズ
float topRecord = 128.0; //動物たちは-z方向に歩いていることに注意
int timerDivisor = 0;
int time = 0;
int generation = 0;
int sequence = 0;
float[dogNum] preRecords; //前回の移動距離を保存しておき，突然変異によってより大きく移動すれば突然変異体を採用
float[string][20][dogNum] preDNAs; //for simple GA

extern (C) void tick(){


	if(timerDivisor++ == 6){
		sequence = (sequence+1)%20;
		timerDivisor = 0;

		switch(strategy){
			case 0: //学習なし
				foreach(elem; chorodogs) elem.move(sequence);
				break;
			case 1: //振動子モデルを使わないDE
				if(!evaluation) foreach(elem; chorodogs) elem.move(sequence);
				else foreach(elem; evaluateds) elem.move(sequence);
				break;
			case 2: //simple GA
				foreach(elem; chorodogs) elem.move(sequence);
				break;
			case 3: //振動子モデル+DE
				if(!evaluation) foreach(elem; chorodogs) elem.move(sequence);
				else foreach(elem; evaluateds) elem.move(sequence);
				break;
			default: break;
		}

	}

		time++;

	//世代終わり
	if(time == 100 + generation*5){


		float proRecordTmp = 0.0; //この世代の最高移動距離

		if(strategy!=0){

			if(!evaluation){
				//評価フェイズ
				if(strategy==3) writeln("start evaluation : ", generation);
			}else{
				writeln("start generation : ", ++generation);
			}


		}


		time = 0;

		switch(strategy){

			//through
			case 0: break;

			//DE
			case 1:

				if(!evaluation){

					foreach(int i, dog; chorodogs){
						preRecords[i] = dog.parts[measuredPart].getZpos();
						preDNAs[i] = dog.dna;
					}

					//犬のリセット
					foreach(int i, ref elem; chorodogs){
						if(proRecordTmp>elem.parts[measuredPart].getZpos()) proRecordTmp = elem.parts[measuredPart].getZpos();
						elem.despawn();
					}
					float ditherF = uniform(0.5f, 1.0f, rnd);
					evaluateds = diffEvo( chorodogs, 0.9f, ditherF);
					evaluation = true;

				}else{
					//犬のリセット
					foreach(int i, ref elem; chorodogs){
						if(proRecordTmp>evaluateds[i].parts[measuredPart].getZpos()) proRecordTmp = evaluateds[i].parts[measuredPart].getZpos();
						elem = new chorodog(to!float(i)*5.0f, 0.0f, 0.0f, true);
						if(evaluateds[i].parts[measuredPart].getZpos() <= preRecords[i]) elem.dna = evaluateds[i].dna;
						else elem.dna = preDNAs[i];
					}
					foreach(int i, ref elem; evaluateds) elem.despawn();
					evaluation = false;
				}

				if(proRecordTmp<topRecord){
					topRecord = proRecordTmp;
					writeln("new record! : ", -1.0*topRecord);
				}


				break;


				//simple GA
			case 2:

				//前回の優秀個体を残す
				float[string][20] pre1FirstDNA = chorodogs[0].dna;
				float[string][20] pre1SecondDNA = chorodogs[5].dna;
				float[string][20] pre1ThirdDNA = chorodogs[10].dna;


				//成績順にソート
				sort!((a,b)=>a.parts[measuredPart].getZpos() > b.parts[measuredPart].getZpos())(chorodogs);
				//優秀なDNAをコピー
				float[string][20] firstDNA = chorodogs[$-1].dna;
				float[string][20] secondDNA = chorodogs[$-2].dna;
				float[string][20] thirdDNA = chorodogs[$-3].dna;

				//新記録を更新したDNAを表示
				if(topRecord > chorodogs[$-1].parts[measuredPart].getZpos()){
					topRecord = -1.0 * chorodogs[$-1].parts[measuredPart].getZpos();
					writeln("New Proceeding Record!: " ~ to!string(topRecord));
				}


				//犬のリセット
				foreach(elem; chorodogs) elem.despawn();
				foreach(int i, ref elem; chorodogs){
					elem = new chorodog(to!float(i)*5.0f, 0.0f, -1.0f, true);
				}

				foreach(int j, dog; chorodogs){

					//最初の2個体はさっきの優秀個体をそのまま動かす
					if( (j >= 0)&&(j < 5)) dog.dna = firstDNA;
					else if( (j >= 5)&&(j < 10) ) dog.dna = secondDNA;
					else if( (j >= 10)&&(j < 15)) dog.dna = thirdDNA;
					else if( (j >= 15)&&(j < 20) ) dog.dna = pre1FirstDNA;
					else if( (j >= 20)&&(j < 25) ) dog.dna = pre1SecondDNA;
					else if( (j >= 25)&&(j < 30) ) dog.dna = pre1ThirdDNA;

					else if(j >= 30){
						//交配
						foreach(int i, dnas; dog.dna){
							foreach(string s, eachdna; dnas){
								if(uniform(0, 2, rnd) == 0) eachdna = firstDNA[i][s];
								else eachdna = secondDNA[i][s];

								//突然変異
								int proOfAttack = uniform(0, 101, rnd);

								if(proOfAttack > attackProbability) eachdna = uniform(-PI/2, PI/2, rnd);
							}
						}
					}

				}

				break;


			//振動子モデル+DE
			case 3:

				if(!evaluation){ //各の移動距離を測るフェイズ

					//geneにはtoString()が(中途半端に)実装されている
					//chorodogs[0].gene.toString();

					foreach(int i, ref elem; chorodogs){

						//移動距離を記録
						preRecords[i] = elem.parts[measuredPart].getZpos();
						//今回の最高記録
						if(proRecordTmp>elem.parts[measuredPart].getZpos()) proRecordTmp = elem.parts[measuredPart].getZpos();
						//chorodogは一旦退場
						elem.despawn();

					}
					//DEに用いるパラメータ
					float ditherF = uniform(0.5f, 1.0f, rnd);

					if(generation==0){ //最初に評価用の犬たちevaluatedsをつくる
						evaluateds.length = dogNum;
						foreach(int i, ref elem; evaluateds) elem = new chorodog(to!float(i)*5.0f, 0.0f, -1.0f, true);
					}else{
						//突然変異
						evolve(evaluateds, chorodogs, 0.9f, ditherF);
						//evaluatedsをpop
						foreach(int i, ref elem; evaluateds) elem.spawn(createVec3(to!float(i)*5.0f, 0.0f, 0.0f));
					}
					evaluation = true; //次は突然変異体評価フェイズ

				}else{ //突然変異体評価フェイズ

					//evaluateds[0].gene.toString();

					foreach(int i, ref elem; evaluateds){
						//今回の最高記録
						if(evaluateds[i].parts[measuredPart].getZpos() < proRecordTmp){
							proRecordTmp = evaluateds[i].parts[measuredPart].getZpos();
						}
						//もし突然変異した各個体が前回の同じindexの個体より良い性能なら採用
						if(evaluateds[i].parts[measuredPart].getZpos() <= preRecords[i]){
							chorodogs[i].gene = elem.gene;
						}
					}
					//突然変異体は一旦退場
					foreach(int i, ref elem; evaluateds) elem.despawn();
					foreach(int i, ref elem; chorodogs) elem.spawn(createVec3(to!float(i)*5.0f, 0.0f, 0.0f));
					evaluation = false; //次は採用した突然変異体を混ぜて性能評価

				}

				//最高記録を表示
				if(proRecordTmp<topRecord){
					topRecord = proRecordTmp;
					writeln("new record! : ", -1.0*topRecord);
				}


				break;

			default: assert(0);

		}



	}




}



//------------------------------------------
