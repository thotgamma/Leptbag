import core.runtime;
import std.stdio;
import std.math;
import std.json;
import std.random;
import std.conv;
import std.algorithm;
import std.array;

import japariSDK.japarilib;
import dlib.math.vector;
import dlib.math.quaternion;

import agent;
import DEforSOG;
import params;
import loadJson;


const int agentNum = 60;
const int averageOf = 3; //一世代averageOf回の試行を行いその平均をスコアとする
const float bodyMass = 10.0f; //動物の総体重．blender側では各パーツに百分率で質量を付与．
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
	loadG6dof(info.g6dofParams);

	writeln("loaded data from .json");

	info.partParams = info.partParams.rehash;
	info.g6dofParams = info.g6dofParams.rehash;


	//agents生成
	agents.length = agentNum*averageOf;

	prepareAgentsGroup(agents, info);

	writeln("made main groups of ", averageOf, " (", agentNum, " agents in each group)");


	for(int i=0; i<agentNum; i++){
		for(int j=1; j<averageOf; j++){
			agents[agentNum*j+i].copyGene(agents[i]);
		}
	}

	writeln("shared gene among main groups");

	writeln("start simulation");

}

void prepareAgentsGroup(agent[] group, agentBodyParameter information){
	//group.length = agentNum*averageOf;

	agent.registerParameter(information);

	foreach(int i, ref elem; group){
		group[i] = new agent(to!float(i)*personalSpace, 0.0f, -1.0f, measuredPart);
	}

}


//毎ステップ実行される--------------------

float topScore = -1000.0f; //動物たちは-z方向に歩いていることに注意
/+
float[] preScores; //突然変異によってより大きく移動すれば突然変異体を採用
float[] evaluatedsScores; //突然変異群のスコアを管理
+/

//そのステップ内で行うべき処理を決定するための変数
int time = 0; //時計
int timerDivisor = 0; //定期的に実行する処理のためのカウンタ
const int moveSpan = 12; //timerDivisorがこの値になるごとに上記処理実行

const int generationStroke = 0; //一世代毎にgenerationStrokeだけ長い時間の試行を行うようになる
const int trialSpan = 500; //一試行の長さ

int generation = 0; //世代を記録する
bool evaluation = false; //trueならDEの突然変異体評価フェイズ
float Cr = 0.9f; //Crの確率で親の遺伝子を引き継ぐ
float coinForRandomMutation = 0.1f; //(1.0-Cr)*coinForRandomMutationの確率で遺伝子要素がランダムに突然変異．
float coeffWind = 0.05f;

extern (C) void tick(){

	time++;

	if(time%2==0){
		timerDivisor++;
		//運動する
		updateAgentsClock();
		//writeln("clock : ", agents[0].biologicalClock);
		//writeln("sequence : ", agents[0].sequenceOfOrder);
	}

	if(time%12==0){
		//運動する
		moveAgents();
	}

	//一世代終了
	if( time == (trialSpan + generation*generationStroke) ){

		writeln();

		time = 0;
		timerDivisor = 0;
		terminateTrial();
	}

}


//----------------------------------------

void updateAgentsClock(){

	//writeln(seq);
	if(!evaluation){
		foreach(elem; agents){
			elem.updateBiologicalClock();
		}
	}else{
		foreach(elem; evaluateds){
			elem.updateBiologicalClock();
		}
	}

}

void moveAgents(){

	//writeln(seq);
	if(!evaluation){
		foreach(elem; agents){
			elem.moveWithSerialOrder();
		}
	}else{
		foreach(elem; evaluateds){
			elem.moveWithSerialOrder();
		}
	}

}


//一試行が終わるたびに実行する処理
void terminateTrial(){


	float proScoreTmp = -1000.0f; //この世代の最高移動距離
	float[] averageScore;
	averageScore.length = averageOf;
	averageScore[] = 0.0f;

	/+
	if(generation==0){
		preScores.length = agentNum*averageOf;
		evaluatedsScores.length = agentNum*averageOf;
	}
	preScores[] = 0.0f;
	evaluatedsScores[] = 0.0f;
	+/

	if(!evaluation){ //各個体の移動距離を測るフェイズ

		foreach(int i, ref elem; agents){

			/+
			//移動距離を記録
			preScores[i] += -1.0f*(elem.parts[measuredPart].getPos().z);
			preScores[i] -= coeffWind * abs(elem.initialPos.x - elem.parts[measuredPart].getPos().x);
			+/


			agents[i].addCurrentPosition(measuredPart);
			agents[i].absScore(true, false, false);



			/+
			for(int k=1; k<=averageOf; k++){
				if(i<agentNum*k){
					averageScore[k-1] += -1.0f*(elem.parts[measuredPart].getPos().z);
					break;
				}
			}
			+/

			proScoreTmp = max( -1.0*agents[i].score.z, proScoreTmp );


		}

		displayGenerationResult(agents, proScoreTmp);

	}else{ //突然変異体評価フェイズ

		//evaluateds[0].gene.toString();

		foreach(int i, ref elem; evaluateds){

			/+
			evaluatedsScores[i] += -1.0f*(elem.parts[measuredPart].getPos().z);
			evaluatedsScores[i] -= coeffWind * abs( elem.initialPos.x - elem.parts[measuredPart].getPos().x);
			+/
			
			evaluateds[i].addCurrentPosition(measuredPart);
			evaluateds[i].absScore(true, false, false);

			/+
			for(int k=1; k<=averageOf; k++){
				if(i<agentNum*k){
					averageScore[k-1] += -1.0f*(elem.parts[measuredPart].getPos().z);
					break;
				}
			}
			+/

			proScoreTmp = max( -1.0*evaluateds[i].score.z, proScoreTmp );

		}

		displayGenerationResult(evaluateds, proScoreTmp);


	}

	terminateGeneration();

}

//今世代の結果を表示
void displayGenerationResult(agent[] group, float proscoretmp){

	//今回の世代の最高記録
	writeln("	top proceeding of this generation : ", proscoretmp);

	writeln("average scores at each trial");

	/+
	write("scores : [ ");
	for(int i=0; i<agentNum; i++){
		write(group[i].score, ", ");
	}
	writeln(" ] ");
	+/


	writeln( culculateAverage(agents, agentNum, averageOf) );
	/+
	for(int k=0; k<averageOf; k++){

		write("		>trial ", k, " : ");
		//今試行の平均移動距離を表示
		writeln(averagescore[k]/to!float(agentNum));

	}
	+/

	//最高記録が出たら記録，表示
	if(proscoretmp>topScore){
		topScore = proscoretmp;
		writeln("!	top proceeding ever! : ", topScore);
	}


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

		/+
		for(int k=1; k<averageOf; k++){
			preScores[0..agentNum] += preScores[agentNum*k..agentNum*(k+1)];
		}
		+/
		Vector3f[] scores = agent.sumScoreOnIndividual(agents, agentNum, averageOf);


		//agentsは一旦退場
		foreach(int i, elem; agents){
			elem.despawn();
		}


		float[] value = culculateValue(scores);
		int[] bests = chooseBest(value);
		/+
		//最良3個体を記録
		int[] bests = [ 0, 0, 0 ];
		float[] scoresTmp;
		scoresTmp.length = agentNum;
		scoresTmp[] = preScores[0..agentNum];
		sort!("a > b")(scoresTmp);
		for(int i=0; i<agentNum; i++){
			if( preScores[i] == scoresTmp[0] ) bests[0] = i;
			else if( preScores[i] == scoresTmp[1] ) bests[1] = i;
			else if( preScores[i] == scoresTmp[2] ) bests[2] = i;
		}
		+/


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
				elem = new agent(to!float(i)*personalSpace, 0.0f, 0.0f, measuredPart);
				elem.copyGene(agents[i]);
			}

			writeln("made groups for evaluation to mutation");
			writeln("copied gene of main group to group for evaluation");

		}else{ //1世代以降

			//evaluatedsをpop
			foreach(int i, ref elem; evaluateds){
				elem.spawn(Vector3f(to!float(i)*personalSpace, 0.0f, 0.0f), measuredPart);
			}

		}

		//DEに用いるパラメータ
		auto rnd = Random(unpredictableSeed);
		float ditherF = uniform(0.0f, 0.5f, rnd);
		//突然変異
		evolveSOG(agentNum, evaluateds[0..agentNum], agents[0..agentNum], coinForRandomMutation, Cr, ditherF, bests);


		for(int i=0; i<agentNum; i++){
			for(int j=1; j<averageOf; j++){
				evaluateds[agentNum*j+i].copyGene(evaluateds[i]);
			}
		}

		evaluation = true; //次は突然変異体評価フェイズ

	}else{ //突然変異体を評価するフェイズ

		float employmentRate = 0.0f; //突然変異個体採用率


		/+
		write("scores : [ ");
		for(int i=0; i<agentNum*averageOf; i++){
			write(agents[i].score, ", ");
		}
		writeln(" ] ");
		+/

		Vector3f[] scoresMain = agent.sumScoreOnIndividual(agents, agentNum, averageOf);
		//writeln("scoresMain", scoresMain);
		float[] valueMain = culculateValue(scoresMain);
		//writeln("valueMain", valueMain);

		Vector3f[] scoresEval = agent.sumScoreOnIndividual(evaluateds, agentNum, averageOf);
		float[] valueEval = culculateValue(scoresEval);

		for(int i=0; i<agentNum; i++){

			//もし突然変異した各個体が前回の同じindexの個体より良い性能なら採用
			//writeln("evalScore[", i, "]=", evaluatedsScores[i], ", preScore[", i, "]=", preScores[i]);
			Random rnd = Random(unpredictableSeed);
			float coin = uniform(0.0f, 1.0f, rnd);
			if( (coin < 0.1f)||( valueEval[i] > valueMain[i] ) ){//  evaluatedsScores[i] > preScores[i] ) ){
				employmentRate += 1.0f;
				agents[i].copyGene(evaluateds[i]);
			}

		}

		for(int i=0; i<agentNum; i++){
			for(int j=1; j<averageOf; j++){
				agents[agentNum*j+i].copyGene(agents[i]);
			}
		}


		writeln("employment rate of the evaluateds : ", employmentRate/to!float(agentNum));

		//突然変異体は一旦退場
		foreach(int i, ref elem; evaluateds) elem.despawn();
		foreach(int i, ref elem; agents) elem.spawn(Vector3f(to!float(i)*personalSpace, 0.0f, 0.0f), measuredPart);

		evaluation = false; //次は採用した突然変異体を混ぜて性能評価

	}


}
