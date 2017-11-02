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

import params;
import loadJson;

Random rnd;


const float bodyMass = 10.0f; //動物の総体重．blender側では各パーツに百分率で質量を付与．



class agent{

	static agentBodyParameter bodyInformation;
	static Vector3f scoreCoeff = Vector3f(-0.3f, 0.0f, -1.0f);
	Vector3f initialPos;
	Vector3f gravityDirection;
	Vector3f eyeDirection;
	elementNode[string] parts;
	generic6DofConstraint[string] g6dofs;
	serialOrderGene SOG;
	int sequenceOfOrder; //SOG.tracksのsequenceOfOrder番目の命令を動作に用いる
	int biologicalClock; //現在のsequenceOfOrderになってからどのくらい時間が経ったか
	Vector3f score; //行動評価


	static void registerParameter(agentBodyParameter info);
	static Vector3f[] culculateAverage(agent[] agents, int agentNum, int averageOf);
	static Vector3f[] sumScoreOnIndividual(agent[] agents, int agentNum, int averageOf);
	static float[] culculateValue(Vector3f scores);
	static void resetAllScores(agent[] agents);
	static void shareGeneAmongGroup(agent[] agents, int agentNum, int averageOf);
	static void evaluationOfEvolution(agent[] agents, agent[] evaluateds, int agentNum, int averageOf);
	static void sortAgentsOnScore(agent[] agents);
	static void swapTracks(agent one, agent two);
	static void swapScores(agent one, agent two);


	this(float x, float y, float z, string measuredPart){

		this.SOG.init();
		this.spawn(Vector3f(x, y, z), measuredPart);

		foreach(string s, dof; g6dofs){
			if(agent.bodyInformation.g6dofParams[s].enabled){
				this.SOG.init(s, agent.bodyInformation.g6dofParams[s].angLimitLower, agent.bodyInformation.g6dofParams[s].angLimitUpper);
			}
		}

	}


	void spawn(Vector3f position, string measuredPart){

		this.initialPos = position;
		this.gravityDirection = Vector3f(0.0f, -1.0f, 0.0f);
		this.eyeDirection = Vector3f(0.0f, 0.0f, -1.0f);

		this.sequenceOfOrder = 0;
		this.biologicalClock = 0;

		agent.bodyInformation.partParams["anker"].mass = 0.0f;

		//身体パーツ
		foreach(string s, partsGen; agent.bodyInformation.partsGenerator){

			parts[s] = partsGen.generate(
					parameterPack(
						param("position", (agent.bodyInformation.partParams[s].position + position) ),
						param("scale",    agent.bodyInformation.partParams[s].scale),
						param("rotation", agent.bodyInformation.partParams[s].rotation),
						param("model",    agent.bodyInformation.partParams[s].vertices),
						param("mass", agent.bodyInformation.partParams[s].mass * bodyMass)
						)
					);

		}

		foreach(part; parts) part.setFriction(this.SOG.friction);

		//6Dof
		foreach(string s, param; agent.bodyInformation.g6dofParams){
			if(agent.bodyInformation.g6dofParams[s].enabled){
				g6dofs[s] = new generic6DofConstraint(
						parts[agent.bodyInformation.g6dofParams[s].object1Name],
						parts[agent.bodyInformation.g6dofParams[s].object2Name],
						agent.bodyInformation.g6dofParams[s].object1Position,
						agent.bodyInformation.g6dofParams[s].object2Position,
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

			this.score = -1.0f * this.parts[measuredPart].getPos();

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

		bool hasSameTracks(agent u){
			foreach(int i, c; this.SOG.tracks){
				foreach(string s, r; c){
					if(this.SOG.tracks[i][s]!=u.SOG.tracks[i][s]){
						return false;
					}
				}
			}
			return true;
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


		void moveManually(int clock){

			if(g6dofs.length==0) return;


			foreach(string s, dof; g6dofs){

				Vector3f currentAngle = Vector3f(dof.getAngle(0), dof.getAngle(1), dof.getAngle(2));
				//writeln("currentAngle : ", currentAngle);
				float goalAngle = -3.141592f + 3.141592f/2.0f*to!float((clock+2)%4);


				Vector3f goal = Vector3f(goalAngle, 0.0f, 0.0f);
				//writeln("goal : ", goal);
				//writeln("\t",3.141592f/2.0f*to!float(clock) );
				Vector3f maxVel = Vector3f(10.0f, 10.0f, 10.0f);

				dof.setRotationalTargetVelocity(
						Vector3f(
							maxVel.x
							*(goal.x - currentAngle.x),
							maxVel.y
							*(goal.y - currentAngle.y),
							maxVel.z
							*(goal.z - currentAngle.z)
							)
						);
			}

		}

		void moveWithSerialOrder(){

			if(g6dofs.length==0) return;


			foreach(string s, dof; g6dofs){

				//getAngle()は[-pi, pi]を返す
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

		void resetScore(string measuredPart){
			this.score = -1.0f*this.parts[measuredPart].getPos();
		}

		void absScore(bool x, bool y, bool z){
			if(x){ this.score.x = abs(this.score.x); }
			if(y){ this.score.y = abs(this.score.y); }
			if(z){ this.score.z = abs(this.score.z); }
		}

		void addCurrentPosition(string measuredPart){
			//writeln(this.score, " : ", this.parts[measuredPart].getPos());
			this.score += this.parts[measuredPart].getPos();
		}

	}


	static void registerParameter(agentBodyParameter info){

		agent.bodyInformation = info;
		foreach(string name, params;agent.bodyInformation.partParams){
			agent.bodyInformation.partsGenerator[name] = new elementManager(agent.bodyInformation.partParams[name].vertices, &createConvexHullShapeBody);
		}

	}

	static Vector3f[] culculateAverage(agent[] agents, int agentNum, int averageOf){
		Vector3f[] average;
		average.length = averageOf;

		for(int i=0; i<averageOf; i++){
			average[i] = Vector3f(0.0f, 0.0f, 0.0f);
			for(int j=0; j<agentNum; j++){
				average[i] += agents[i*agentNum+j].score;
			}
			average[i] /= agentNum;
		}

		return average;
	}


	static Vector3f[] sumScoreOnIndividual(ref agent[] agents, int agentNum, int averageOf){
		Vector3f[] scores;
		scores.length = agentNum;

		for(int i=0; i<agentNum; i++){
			scores[i] = Vector3f(0.0f, 0.0f, 0.0f);
			for(int j=0; j<averageOf; j++){
				scores[i] += agents[j*agentNum+i].score;
			}
		}

		return scores;
	}

	static int[] chooseBest(float[] value){
		int[] bests;
		bests.length = 3;
		bests[] = 0;

		float[] valueTmp;
		valueTmp.length = value.length;
		valueTmp[] =value[];
		sort!("a > b")(valueTmp);
		for(int i=0; i<value.length; i++){
			if( value[i] == valueTmp[0] ){ bests[0] = i; }
			else if( value[i] == valueTmp[1] ){ bests[1] = i; }
			else if( value[i] == valueTmp[2] ){ bests[2] = i; }
		}

		return bests;
	}

	static float[] culculateValue(Vector3f[] scores){
		float[] value;
		value.length = scores.length;
		value[] = 0.0f;

		for(int i=0; i<value.length; i++){
			value[i] = agent.scoreCoeff.x * scores[i].x + agent.scoreCoeff.y * scores[i].y + agent.scoreCoeff.z * scores[i].z;
		}

		return value;
	}

	static void resetAllScores(agent[] agents, string measuredPart){
		foreach(int i, ref elem; agents){
			agents[i].resetScore(measuredPart);
		}
	}


	static void shareGeneAmongGroup(ref agent[] agents, int agentNum, int averageOf){

		for(int i=0; i<agentNum; i++){
			for(int j=1; j<averageOf; j++){
				agents[agentNum*j+i].copyGene(agents[i]);
			}
		}

	}


	static void evaluateEvolution(ref agent[] agents, ref agent[] evaluateds, int agentNum, int averageOf){

		float employmentRate = 0.0f; //突然変異個体採用率

		Vector3f[] scoresMain = sumScoreOnIndividual(agents, agentNum, averageOf);
		float[] valueMain = culculateValue(scoresMain);

		Vector3f[] scoresEval = sumScoreOnIndividual(evaluateds, agentNum, averageOf);
		float[] valueEval = culculateValue(scoresEval);

		for(int i=0; i<agentNum; i++){

			//もし突然変異した各個体が前回の同じindexの個体より良い性能なら採用
			Random rnd = Random(unpredictableSeed);
			float coin = uniform(0.0f, 1.0f, rnd);
			if( (coin < 0.1f)||( valueEval[i] > valueMain[i] ) ){
				employmentRate += 1.0f;
				agents[i].copyGene(evaluateds[i]);
				agents[i].score = evaluateds[i].score;
			}

		}
		writeln("\temployment rate of the evaluateds : ", employmentRate/to!float(agentNum));

		sortAgentsOnScoreZ(agents, agentNum, averageOf);
		shareGeneAmongGroup(agents, agentNum, averageOf);

	}


	static void sortAgentsOnScoreZ(ref agent[] agents, int agentNum, int  averageOf){

		/+
			writeln("buma");
		writeln(agents[0].score.z);
		writeln(agents[0].SOG.tracks);
		writeln(agents[agentNum].score.z);
		writeln(agents[agentNum].SOG.tracks);
		writeln(agents[agentNum*2].score.z);
		writeln(agents[agentNum*2].SOG.tracks);
		+/
			/+
			for(int i=0; i<agentNum; i++){
				for(int j=1; j<averageOf; j++){
					agents[i].score += agents[j*agentNum+i].score;
				}
			}
		+/

		float[] scoreZ;
		scoreZ.length = agentNum;
		for(int i=0; i<scoreZ.length; i++){
			scoreZ[i] = agents[i].score.z;//scores[i].z;
		}
		sort!("a < b")(scoreZ);

		//i番目のスコアをもつagentのindexを取得
		//そのスコアをもつagentをi番目に格納．i番目にいたagentはそのagentがいた場所に．
		for(int i=0; i<agentNum-1; i++){

			int index = -1;
			for(int j=0; j<agentNum; j++){
				if(scoreZ[i]==agents[j].score.z){
					index = j;
					break;
				}
			}

			swapTracks(agents[i], agents[index]);
			swapScores(agents[i], agents[index]);
		}

		/+
			for(int i=0; i<agentNum-1; i++){
				if(agents[i].score.z==agents[i+1].score.z){
					writeln("bumanyan");
				}
			}

		for(int i=0; i<agentNum; i++){
			for(int j=0; j<agentNum; j++){
				if(i!=j){
					if(scoreZ[i]==scoreZ[j]){
						writeln("exist same");
					}
				}
			}
		}
		+/
			/+
			for(int i=0; i<scoreZ.length; i++){
				if(scoreZ[i]!=agents[i].score.z){
					writeln("tsurakibi");
					assert(0);
				}
			}

		+/
			/+
			for(int i=0; i<agentNum-1; i++){
				if(agents[i].score.z>agents[i+1].score.z){
					writeln(i);
					writeln(agents[i].score.z, " : ", agents[i+1].score.z);
					writeln("assert(0)");
					assert(0);
				}
			}
		+/

			writeln("\tsorted agents on their evaluated score.z");

	}


	static void swapTracks(agent one, agent two){
		Vector3f[string][] tmp;
		tmp = one.SOG.tracks;
		one.SOG.tracks = two.SOG.tracks;
		two.SOG.tracks = tmp;

	}

	static void swapScores(agent one, agent two){
		Vector3f tmp;
		tmp = one.score;
		one.score = two.score;
		two.score = tmp;
	}
