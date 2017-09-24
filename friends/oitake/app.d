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

Random rnd;

const int agentNum = 100;

string measuredPart = "head"; //この名前のパーツの移動距離を測る
const float bodyMass = 50.0f; //動物の総体重．blender側では各パーツに百分率で質量を付与．
const float personalSpace = 5.0f; //動物を並べる間隔
agent[] agents; //メインとする群
agent[] evaluateds; //DEにおける突然変異個体
agentBodyParameter info;





//ApplicationInterface----------------------


//initialize--------------------------------

extern (C) void init(){
	rt_init();
	Random(unpredictableSeed);
	writeln("model:oitake loaded");


	//jsonからload
	loadMesh(info.partParams);
	loadHinge(info.hingeParams);
	loadG6dof(info.g6dofParams);


	info.partParams = info.partParams.rehash;
	info.hingeParams = info.hingeParams.rehash;
	info.g6dofParams = info.g6dofParams.rehash;


	//agents生成
	agents.length = agentNum;
	foreach(int i, ref elem; agents){
		elem = new agent(to!float(i)*personalSpace, 0.0f, -1.0f, info);
	}


	//最初が0世代目
	writeln("start generation : 0");

}



//毎ステップ実行される--------------------


float topScore = 0.0f; //動物たちは-z方向に歩いていることに注意
float[agentNum] preScores = 0.0f; //突然変異によってより大きく移動すれば突然変異体を採用
float[agentNum] evaluatedsScores = 0.0f; //突然変異群のスコアを管理

//そのステップ内で行うべき処理を決定するための変数
int time = 0; //時計
int timerDivisor = 0; //定期的に実行する処理のためのカウンタ
const int moveSpan = 16; //timerDivisorがこの値になるごとに上記処理実行

const int generationStroke = 0; //一世代毎にgenerationStrokeだけ長い時間の試行を行うようになる
const int trialSpan = 500; //一試行の長さ
const int averageOf = 3; //一世代averageOf回の試行を行いその平均をスコアとする

int sequence = 0;
int trial = 0; //試行のカウンタ
int generation = 0; //世代を記録する
bool evaluation = false; //trueならDEの突然変異体評価フェイズ

extern (C) void tick(){

	time++;

	//運動する
	if(timerDivisor<2){
		moveAgents();
	}

	//運動命令変化
	if(timerDivisor++ == moveSpan){
		timerDivisor = 0;
		regularProcess();
	}

	//一試行終了
	if(time == (trialSpan + generation*generationStroke)){
		time = 0;
		timerDivisor = 0;
		terminateTrial();
		trial++;
	}

	//一世代終了
	if(trial==averageOf){
		trial = 0;
		timerDivisor = 0;
		terminateGeneration();
	}

}


//----------------------------------------


void moveAgents(){

	//writeln(sequence);
	if(!evaluation) foreach(elem; agents) elem.moveWithSerialOrder(sequence);
	else foreach(elem; evaluateds) elem.moveWithSerialOrder(sequence);
}

//定期的に実行する処理
void regularProcess(){
	//writeln(sequence);
	sequence = (sequence+1)%5;
}


//一試行が終わるたびに実行する処理
void terminateTrial(){

	sequence = 0;

	writeln("\n>		trial at : ", trial, "-----\n");

	float proScoreTmp = 0.0f; //この世代の最高移動距離
	float averageScore = 0.0f;

	if(!evaluation){ //各個体の移動距離を測るフェイズ

		//geneにはtoString()が(中途半端に)実装されている
		//agents[0].gene.toString();

		foreach(int i, ref elem; agents){

			//移動距離を記録
			preScores[i] += elem.parts[measuredPart].getZpos();
			averageScore += elem.parts[measuredPart].getZpos();

			//今回の最高記録(-z方向が前)
			proScoreTmp = min( elem.parts[measuredPart].getZpos(), proScoreTmp );

			//初期位置に戻る
			elem.despawn();
			elem.spawn(createVec3(to!float(i)*personalSpace, 0.0f, 0.0f));

		}

	}else{ //突然変異体評価フェイズ

		//evaluateds[0].gene.toString();

		foreach(int i, ref elem; evaluateds){

			evaluatedsScores[i] += elem.parts[measuredPart].getZpos();
			averageScore += elem.parts[measuredPart].getZpos();

			//今回の最高記録
			proScoreTmp = min( evaluateds[i].parts[measuredPart].getZpos(), proScoreTmp );

			//初期位置に戻る
			elem.despawn();
			elem.spawn(createVec3(to!float(i)*personalSpace, 0.0f, 0.0f));

		}

	}


	//最高記録が出たら記録，表示
	if(proScoreTmp<topScore){
		topScore = proScoreTmp;
		writeln("\n!		top score ever! : ", -1.0f*topScore);
	}

	//今回の試行の最高記録
	writeln("		top score of this trial : ", -1.0f*proScoreTmp);

	//今試行の平均移動距離を表示
	writeln("		average score : ", -1.0f*averageScore/to!float(agentNum));



}

//試行がaverageOf回行われるたびに実行される処理
void terminateGeneration(){


	if(!evaluation){ //評価フェイズ
		writeln("	start evaluation : ", generation);
	}else{
		writeln("start generation : ", ++generation, "----------------");
	}


	if(!evaluation){ //各個体の移動距離を測るフェイズ


		//最良3個体を記録
		int[] bests = [ 0, 0, 0 ];
		foreach(int i, ref elem ;agents){

			//agentは一旦退場
			elem.despawn();

			if( preScores[bests[0]] > preScores[i] ){
				bests[2] = bests[1];
				bests[1] = bests[0];
				bests[0] = i;
			}else if( preScores[bests[1]] > preScores[i] ){
				bests[2] = bests[1];
				bests[1] = i;
			}else if( preScores[bests[2]] > preScores[i] ){
				bests[2] = i;
			}


		}

		//writeln("preScores: ", preScores);
		//for(int k=0; k<3; k++) writeln(bests[k], " : ", preScores[bests[k]]);



		if(generation==0){ //最初に評価用の犬たちevaluatedsをつくる
			evaluateds.length = agentNum;
			foreach(int i, ref elem; evaluateds) elem = new agent(to!float(i)*personalSpace, 0.0f, 0.0f, info);

			//DEに用いるパラメータ
			float ditherF = uniform(0.5f, 1.0f, rnd);
			//突然変異
			evolveBest(evaluateds, agents, 0.1f, ditherF, bests);
			evolveSOG(evaluateds, agents, 0.1f, ditherF, bests);





		}else{ //0世代以降は突然変異を行う



			//evaluatedsをpop
			foreach(int i, ref elem; evaluateds) elem.spawn(createVec3(to!float(i)*personalSpace, 0.0f, 0.0f));

			//DEに用いるパラメータ
			float ditherF = uniform(0.5f, 1.0f, rnd);
			//突然変異
			evolveBest(evaluateds, agents, 0.1f, ditherF, bests);
			evolveSOG(evaluateds, agents, 0.1f, ditherF, bests);

		}

		/*
		writeln("buma");
		writeln("child");
		foreach(string s, dof; evaluateds[0].g6dofs){
			write(s, " ( ");
			for(uint i=0; i<evaluateds[0].SOG.lengthOfSet; i++){
				write(i, ": ", evaluateds[0].SOG.tracks[i][s].getx(), ", ");
			}
			writeln(")");
		}

		writeln("1");
		foreach(string s, dof; agents[0].g6dofs){
			write(s, " ( ");
			for(uint i=0; i<agents[0].SOG.lengthOfSet; i++){
				write(i, ":", agents[bests[0]].SOG.tracks[i][s].getx(), ", ");
			}
			writeln(")");
		}

		writeln("2");
		foreach(string s, dof; agents[0].g6dofs){
			write(s, " ( ");
			for(uint i=0; i<agents[0].SOG.lengthOfSet; i++){
				write(i, ":", agents[bests[1]].SOG.tracks[i][s].getx(), ", ");
			}
			writeln(")");
		}
		*/

		evaluation = true; //次は突然変異体評価フェイズ


	}else{ //突然変異体評価フェイズ

		float employmentRate = 0.0f; //突然変異個体採用率
		foreach(int i, ref elem; evaluateds){

			//もし突然変異した各個体が前回の同じindexの個体より良い性能なら採用
			//writeln("evalScore[", i, "]=", evaluatedsScores[i], "\npreScore[", i, "]=", preScores[i]);
			if(evaluatedsScores[i] < preScores[i]){
				//writeln("buma!");
				employmentRate += 1.0f;
				agents[i].gene = elem.gene;
				agents[i].SOG.tracks = elem.SOG.tracks;
			}


			//スコアリセット
			preScores[i] = 0.0f;
			evaluatedsScores[i] = 0.0f;

		}
		writeln("employment rate of the evaluateds : ", employmentRate/to!float(agentNum));

		//突然変異体は一旦退場
		foreach(int i, ref elem; evaluateds) elem.despawn();
		foreach(int i, ref elem; agents) elem.spawn(createVec3(to!float(i)*personalSpace, 0.0f, 0.0f));

		evaluation = false; //次は採用した突然変異体を混ぜて性能評価

	}





}
