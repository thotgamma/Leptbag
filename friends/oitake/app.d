import core.runtime;
import std.stdio;
import std.math;
import std.json;
import std.random;
import std.conv;
import std.algorithm;
import std.array;

import japariSDK.japarilib;
import agent;
import DEforOscillator2;
import DEforSOG;
import Oscillator;
import params;
import loadJson;


const int agentNum = 50;
const int averageOf = 3; //一世代averageOf回の試行を行いその平均をスコアとする
const float bodyMass = 5.0f; //動物の総体重．blender側では各パーツに百分率で質量を付与．
const float personalSpace = 5.0f; //動物を並べる間隔
const string measuredPart = "head"; //この名前のパーツの移動距離を測る

agent[] agents; //メインとする群
agent[] evaluateds; //DEにおける突然変異個体

agentBodyParameter info;





//ApplicationInterface----------------------


//initialize--------------------------------

extern (C) void init(){

	rt_init();
	Random(unpredictableSeed);
	writeln("start initialization model:oitake");


	//jsonからload
	loadMesh(info.partParams);
	loadHinge(info.hingeParams);
	loadG6dof(info.g6dofParams);

	writeln("loaded data from .json");

	info.partParams = info.partParams.rehash;
	info.hingeParams = info.hingeParams.rehash;
	info.g6dofParams = info.g6dofParams.rehash;


	//agents生成
	agents.length = agentNum*averageOf;
	foreach(int i, ref elem; agents){
		elem = new agent(to!float(i)*personalSpace, 0.0f, -1.0f, info);
	}

	writeln("made main groups of ", averageOf, "(", agentNum, " agent in each group");

	//agents[0].checkSOG();

	for(int i=0; i<agentNum; i++){
		for(int j=1; j<averageOf; j++){
			agents[agentNum*j+i].copyGene(agents[i]);
		}
	}

	writeln("shared gene among main groups");

	writeln("start simulation");

}



//毎ステップ実行される--------------------

float topScore = 0.0f; //動物たちは-z方向に歩いていることに注意
float[] preScores; //突然変異によってより大きく移動すれば突然変異体を採用
float[] evaluatedsScores; //突然変異群のスコアを管理

//そのステップ内で行うべき処理を決定するための変数
int time = 0; //時計
int timerDivisor = 0; //定期的に実行する処理のためのカウンタ
const int moveSpan = 12; //timerDivisorがこの値になるごとに上記処理実行

const int generationStroke = 0; //一世代毎にgenerationStrokeだけ長い時間の試行を行うようになる
const int trialSpan = 500; //一試行の長さ

int generation = 0; //世代を記録する
bool evaluation = false; //trueならDEの突然変異体評価フェイズ
float Cr = 0.5f; //Crの確率で親の遺伝子を引き継ぐ
float coinForRandomMutation = 0.3f; //(1.0-Cr)*coinForRandomMutationの確率で遺伝子要素がランダムに突然変異．

extern (C) void tick(){

	if(time==0) write("progress:");
	else if(time%(trialSpan/9)==0) write("#");

	time++;

	if(time%2==0){
		timerDivisor++;
		//運動する
		moveAgents();
	}


	//一世代終了
	if( time == (trialSpan + generation*generationStroke) ){

		writeln();
		/+
		if(!evaluation){
			writeln(agents[0].parts[measuredPart].getZpos());
			writeln(agents[agentNum].parts[measuredPart].getZpos());
			writeln(agents[2*agentNum].parts[measuredPart].getZpos());

			agents[0].checkSOG();
			agents[agentNum].checkSOG();
			agents[2*agentNum].checkSOG();
		}
		+/

		time = 0;
		timerDivisor = 0;
		terminateTrial();
	}

}


//----------------------------------------


void moveAgents(){

	//writeln(seq);
	if(!evaluation){
		foreach(elem; agents){
			elem.moveWithSerialOrder();
			elem.updateBiologicalClock();
		}
	}else{
		foreach(elem; evaluateds){
			elem.moveWithSerialOrder();
			elem.updateBiologicalClock();
		}
	}

}






//一試行が終わるたびに実行する処理
void terminateTrial(){



	float proScoreTmp = 0.0f; //この世代の最高移動距離
	float[averageOf] averageScore = 0.0f;

	if(generation==0){
		preScores.length = agentNum*averageOf;
		evaluatedsScores.length = agentNum*averageOf;
	}

	if(!evaluation){ //各個体の移動距離を測るフェイズ

		//geneにはtoString()が(中途半端に)実装されている
		//agents[0].gene.toString();



		foreach(int i, ref elem; agents){

			//移動距離を記録
			preScores[i] += elem.parts[measuredPart].getZpos();



			for(int k=1; k<=averageOf; k++){
				if(i<agentNum*k){
					averageScore[k-1] += elem.parts[measuredPart].getZpos();
					break;
				}
			}

			proScoreTmp = min( elem.parts[measuredPart].getZpos(), proScoreTmp );


			/+
			//初期位置に戻る
			elem.despawn();
			elem.spawn(createVec3(to!float(i)*personalSpace, 0.0f, 0.0f));
			+/

		}
		/+
		writeln(averageScore);

		averageScore = 0.0f;
		writeln(preScores);
		for(int k=0; k<averageOf; k++){
			for(int j=0; j<agentNum; j++){
				averageScore[k] += preScores[k*agentNum + j];
			}
		}
		writeln(averageScore);
		+/

	}else{ //突然変異体評価フェイズ

		//evaluateds[0].gene.toString();

		foreach(int i, ref elem; evaluateds){

			evaluatedsScores[i] += elem.parts[measuredPart].getZpos();

			for(int k=1; k<=averageOf; k++){
				if(i<agentNum*k){
					averageScore[k-1] += elem.parts[measuredPart].getZpos();
					break;
				}
			}

			proScoreTmp = min( elem.parts[measuredPart].getZpos(), proScoreTmp );


			/+
			//初期位置に戻る
			elem.despawn();
			elem.spawn(createVec3(to!float(i)*personalSpace, 0.0f, 0.0f));
			+/

		}
		/+
		writeln(averageScore);

		averageScore = 0.0f;
		writeln(preScores);
		for(int k=0; k<averageOf; k++){
			for(int j=0; j<agentNum; j++){
				averageScore[k] += evaluatedsScores[k*agentNum + j];
			}
		}
		writeln(averageScore);
		+/

	}

	//今回の世代の最高記録
	writeln("		top score of this generation : ", -1.0f*proScoreTmp);

	for(int k=0; k<averageOf; k++){

		writeln("\n>		trial ", k, ":\n");
		//今試行の平均移動距離を表示
		writeln("			average score : ", -1.0f*averageScore[k]/to!float(agentNum));

	}


	//最高記録が出たら記録，表示
	if(proScoreTmp<topScore){
		topScore = proScoreTmp;
		writeln("!			top score ever! : ", -1.0f*topScore);
	}

	terminateGeneration();

}


//世代の終了時処理
void terminateGeneration(){


	if(!evaluation){ //評価フェイズ
		writeln("	start evaluation ", generation, ":");
	}else{
		writeln("start generation ", ++generation, ": ---------------------------------");
	}


	if(!evaluation){ //各個体の移動距離を測るフェイズ

		/*
		for(int k=0; k<agentNum; k++){
			for(int l=1; l<averageOf; l++){
				preScores[k] += preScores[agentNum*l+k];
			}
		}
		*/

		for(int k=1; k<averageOf; k++){
			preScores[0..agentNum] += preScores[agentNum*k..agentNum*(k+1)];
		}

		//agentsは一旦退場
		foreach(int i, elem; agents){
			elem.despawn();
		}

		//最良3個体を記録
		int[] bests = [ 0, 0, 0 ];
		float[] scoresTmp;
		scoresTmp.length = agentNum;
		scoresTmp[] = preScores[0..agentNum];
		sort!("a < b")(scoresTmp);
		for(int i=0; i<agentNum; i++){
			if( preScores[i] == scoresTmp[0] ) bests[0] = i;
			else if( preScores[i] == scoresTmp[1] ) bests[1] = i;
			else if( preScores[i] == scoresTmp[2] ) bests[2] = i;
		}

		/+ 
		//最良個体を選べているか確認
		writeln("preScores : ", preScores[0.. agentNum]);
		writeln("sorted preScores : ", scoresTmp);
		for(int k=0; k<3; k++){
			writeln("best", k, " : no.", bests[k], " : ", preScores[bests[k]]);
		}
		+/

		//writeln("preScores: ", preScores);
		//for(int k=0; k<3; k++) writeln(bests[k], " : ", preScores[bests[k]]);

		if(generation==0){ //最初に評価用の犬たちevaluatedsをつくる

			evaluateds.length = agentNum*averageOf;
			foreach(int i, ref elem; evaluateds){
				elem = new agent(to!float(i)*personalSpace, 0.0f, 0.0f, info);
				elem.copyGene(agents[i]);
			}

			writeln("made groups for evaluation to mutation");

			//DEに用いるパラメータ
			auto rnd = Random(unpredictableSeed);
			float ditherF = uniform(0.5f, 1.0f, rnd);
			//突然変異
			//evolveBest(evaluateds[0..agentNum], agents[0..agentNum], 0.1f, ditherF, bests);
			evolveSOG(agentNum, evaluateds[0..agentNum], agents[0..agentNum], coinForRandomMutation, Cr, ditherF, bests);

			for(int i=0; i<agentNum; i++){
				for(int j=1; j<averageOf; j++){
					evaluateds[agentNum*j+i].copyGene(evaluateds[i]);
				}
			}

			writeln("copied gene of main group to group for evaluation");

		}else{ //1世代以降

			//evaluatedsをpop
			foreach(int i, ref elem; evaluateds){
				elem.spawn(createVec3(to!float(i)*personalSpace, 0.0f, 0.0f));
			}

			//DEに用いるパラメータ
			auto rnd = Random(unpredictableSeed);
			float ditherF = uniform(0.5f, 1.0f, rnd);
			//突然変異
			//evolveBest(evaluateds[0..agentNum], agents[0..agentNum], 0.1f, ditherF, bests);

			/+
			writeln("before evolution");
			foreach(int i, elem; evaluateds){
				if(i<1){
					writeln("evaluateds[", i, "]");
					evaluateds[i].checkSOG();
				}
			}

			foreach(int i, elem; agents){
				if(i<1){
					writeln("best agents[", bests[0], "]");
					agents[bests[0]].checkSOG();
				}
			}
			+/

			evolveSOG(agentNum, evaluateds[0..agentNum], agents[0..agentNum], coinForRandomMutation, Cr, ditherF, bests);

			/+
			writeln("after evolution");
			foreach(int i, elem; evaluateds){
				if(i<1){
					writeln("evaluateds[", i, "]");
					evaluateds[i].checkSOG();
				}
			}

			foreach(int i, elem; agents){
				if(i<1){
					writeln("best agents[", bests[0], "]");
					agents[bests[0]].checkSOG();
				}
			}
			+/

			for(int i=0; i<agentNum; i++){
				for(int j=1; j<averageOf; j++){
					evaluateds[agentNum*j+i].copyGene(evaluateds[i]);
				}
			}
		}

		/*
		writeln("buma");
		writeln("child");
		foreach(string s, dof; evaluateds[0].g6dofs){
			write(s, " ( ");
			for(uint i=0; i<evaluateds[0].SOG.tracks.length; i++){
				write(i, ": ", evaluateds[0].SOG.tracks[i][s].getx(), ", ");
			}
			writeln(")");
		}

		foreach(string s, dof; agents[0].g6dofs){
			write(s, " ( ");
			for(uint i=0; i<agents[0].SOG.tracks.length; i++){
				write(i, ":", agents[bests[0]].SOG.tracks[i][s].getx(), ", ");
			}
			writeln(")");
		}

		foreach(string s, dof; agents[0].g6dofs){
			write(s, " ( ");
			for(uint i=0; i<agents[0].SOG.tracks.length; i++){
				write(i, ":", agents[bests[1]].SOG.tracks[i][s].getx(), ", ");
			}
			writeln(")");
		}
		*/

		evaluation = true; //次は突然変異体評価フェイズ


	}else{ //突然変異体を評価するフェイズ

		float employmentRate = 0.0f; //突然変異個体採用率

		for(int i=0; i<agentNum; i++){

			//もし突然変異した各個体が前回の同じindexの個体より良い性能なら採用
			//writeln("evalScore[", i, "]=", evaluatedsScores[i], ", preScore[", i, "]=", preScores[i]);
			Random rnd = Random(unpredictableSeed);
			float coin = uniform(0.0f, 1.0f, rnd);
			if( (coin < 0.1f)||( evaluatedsScores[i] < preScores[i] ) ){
				//writeln("employed");
				employmentRate += 1.0f;
				//agents[i].checkSOG;
				agents[i].copyGene(evaluateds[i]);
				//agents[i].checkSOG;
				//evaluateds[i].checkSOG;
			}

		}

		for(int i=0; i<agentNum; i++){
			for(int j=1; j<averageOf; j++){
				agents[agentNum*j+i].copyGene(agents[i]);
			}
		}


		//スコアリセット
		for(int j=0; j<preScores.length; j++) preScores[j] = 0.0f;
		for(int j=0; j<evaluatedsScores.length; j++) evaluatedsScores[j] = 0.0f;

		writeln("employment rate of the evaluateds : ", employmentRate/to!float(agentNum));

		//突然変異体は一旦退場
		foreach(int i, ref elem; evaluateds) elem.despawn();
		foreach(int i, ref elem; agents) elem.spawn(createVec3(to!float(i)*personalSpace, 0.0f, 0.0f));

		evaluation = false; //次は採用した突然変異体を混ぜて性能評価

	}


}
