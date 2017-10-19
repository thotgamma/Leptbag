import std.stdio;
import std.math;
import std.json;
import std.random;
import std.conv;
import std.algorithm;

import japariSDK.japarilib;
import dlib.math.vector;
import dlib.math.quaternion;

import agent;
import params;

Random rnd;

void simpleSOG(agent[] children, agent[] parents, float Cr, float F, int[] bests){

	auto rnd = Random(unpredictableSeed);

	int k = bests[0];
	int l = bests[1];
	int m = bests[2];

	foreach(int j, child; children){

		foreach(string s, dof; child.g6dofs){
			for(uint i=0; i<child.SOG.tracks.length; i++){

				float coin = uniform(0.0f, 1.0f, rnd);
				//if((j==0)&&(s=="Constraint")) writeln("no. ", i , " : coin is ", coin);
				if(0.4 > coin){
					//if((j==0)&&(s=="Constraint")) writeln("use first gene");
					child.SOG.tracks[i][s] = parents[k].SOG.tracks[i][s];
				}else if(0.8 > coin){
					//if((j==0)&&(s=="Constraint")) writeln("use second gene");
					child.SOG.tracks[i][s] = parents[l].SOG.tracks[i][s];
				}else{
					/+
					if((j==0)&&(s=="Constraint")) writeln("before mutation");
					if((j==0)&&(s=="Constraint")) writeln(child.SOG.tracks[i][s].getx());
					+/

					child.SOG.init(s, child.bodyInformation.g6dofParams[s].angLimitLower, child.bodyInformation.g6dofParams[s].angLimitUpper);
					/+
					if((j==0)&&(s=="Constraint")) writeln("after mutation");
					if((j==0)&&(s=="Constraint")) writeln(child.SOG.tracks[i][s].getx());
					+/
				}


			}
		}
	}

	/+
	writeln("children[0]");
	children[0].checkSOG();
	writeln("best parents");
	parents[bests[0]].checkSOG();
	+/


}


//rand
void evolveSOG(int agentNum, agent[] children, agent[] parents, float coin, float Cr, float F){

	auto rnd = Random(unpredictableSeed);

	foreach(int j, child; children){
		foreach(string s, dof; child.g6dofs){

			int k = uniform(0, agentNum, rnd);
			int l = uniform(0, agentNum, rnd);
			int m = uniform(0, agentNum, rnd);

			if(Cr > uniform(0.0f, 1.0f, rnd)){
				for(uint i=0; i<child.SOG.tracks.length; i++){
					child.SOG.tracks[i][s] = parents[k].SOG.tracks[i][s] + F * ( parents[m].SOG.tracks[i][s] - parents[l].SOG.tracks[i][s] );
				}
			}else{

				if(coin > uniform(0.0f, 1.0f, rnd)){
					for(uint i=0; i<child.SOG.tracks.length; i++){
						child.SOG.init(s, child.bodyInformation.g6dofParams[s].angLimitLower, child.bodyInformation.g6dofParams[s].angLimitUpper);
					}
				}else{
					for(uint i=0; i<child.SOG.tracks.length; i++){
						child.SOG.tracks[i][s] = parents[j].SOG.tracks[i][s];
					}

				}

			}
		}
	}

}


//best
void evolveSOG(int agentNum, agent[] children, agent[] parents, float coin, float Cr, float F, int[] bests){

	auto rnd = Random(unpredictableSeed);

	int k = bests[0];
	int l = bests[1];
	int m = bests[2];

	foreach(int j, child; children){

		foreach(string s, dof; child.g6dofs){

			if(Cr > uniform(0.0f, 1.0f, rnd)){


				for(uint i=0; i<child.SOG.tracks.length; i++){
					child.SOG.tracks[i][s] = parents[k].SOG.tracks[i][s] + F * ( parents[m].SOG.tracks[i][s] - parents[l].SOG.tracks[i][s] );
				}

			}else{


				for(uint i=0; i<child.SOG.tracks.length; i++){
					child.SOG.tracks[i][s] = parents[j].SOG.tracks[i][s];
				}

			}

			for(uint i=0; i<child.SOG.tracks.length; i++){
				if(coin > uniform(0.0f, 1.0f, rnd)){
					child.SOG.init(i, s, child.bodyInformation.g6dofParams[s].angLimitLower, child.bodyInformation.g6dofParams[s].angLimitUpper);
				}
			}


		}


	}


}
