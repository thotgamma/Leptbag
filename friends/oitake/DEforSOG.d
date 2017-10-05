import std.stdio;
import std.math;
import std.json;
import std.random;
import std.conv;
import std.algorithm;

import japariSDK.japarilib;
import agent;
import DEforOscillator2;
import Oscillator;
import params;

Random rnd;

void simpleSOG(agent[] children, agent[] parents, float Cr, float F, int[] bests){

	auto rnd = Random(unpredictableSeed);

	int k = bests[0];
	int l = bests[1];
	int m = bests[2];

	foreach(int j, child; children){

		foreach(string s, dof; child.g6dofs){
			for(uint i=0; i<child.SOG.lengthOfSet; i++){

				float coin = uniform(0.0f, 1.0f, rnd);
				//if((j==0)&&(s=="Constraint")) writeln("no. ", i , " : coin is ", coin);
				if(0.4 > coin){
					//if((j==0)&&(s=="Constraint")) writeln("use first gene");
					child.SOG.tracks[i][s] = createVec3(
							parents[k].SOG.tracks[i][s].getx(),
							parents[k].SOG.tracks[i][s].gety(),
							parents[k].SOG.tracks[i][s].getz(),
							);
				}else if(0.8 > coin){
					//if((j==0)&&(s=="Constraint")) writeln("use second gene");
					child.SOG.tracks[i][s] = createVec3(
							parents[l].SOG.tracks[i][s].getx(),
							parents[l].SOG.tracks[i][s].gety(),
							parents[l].SOG.tracks[i][s].getz(),
							);
				}else{
					/+
					if((j==0)&&(s=="Constraint")) writeln("before mutation");
					if((j==0)&&(s=="Constraint")) writeln(child.SOG.tracks[i][s].getx());
					+/
					child.SOG.tracks[i][s] = createVec3( uniform(-1.57f/2.0f, 1.57f/2.0f, rnd), uniform(-1.57f/2.0f, 1.57f/2.0f, rnd), 0.0f );
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


void evolveSOG(agent[] children, agent[] parents, float coin, float Cr, float F, int[] bests){

	auto rnd = Random(unpredictableSeed);

	foreach(int j, child; children){

		int k = bests[0];
		int l = bests[1];
		int m = bests[2];

		foreach(string s, dof; child.g6dofs){
			for(uint i=0; i<child.SOG.lengthOfSet; i++){

				if(Cr < uniform(0.0f, 1.0f, rnd)){


					child.SOG.tracks[i][s] = createVec3(
							parents[k].SOG.tracks[i][s].getx() + F * ( parents[m].SOG.tracks[i][s].getx() - parents[l].SOG.tracks[i][s].getx() ),
							parents[k].SOG.tracks[i][s].gety() + F * ( parents[m].SOG.tracks[i][s].gety() - parents[l].SOG.tracks[i][s].gety() ),
							parents[k].SOG.tracks[i][s].getz() + F * ( parents[m].SOG.tracks[i][s].getz() - parents[l].SOG.tracks[i][s].getz() )
							);

				}else{

					if(coin > uniform(0.0f, 1.0f, rnd)){
						child.SOG.init(s, child.bodyInformation.g6dofParams[s].angLimitLower, child.bodyInformation.g6dofParams[s].angLimitUpper);
					}else{
						child.SOG.tracks[i][s] = parents[j].SOG.tracks[i][s];
					}

				}

			}

		}

		//writeln("j", j);


		/*
		if(j==0){
			writeln("neko");

			writeln("child");
			foreach(string s, dof; child.g6dofs){
				write(s, " ( ");
				for(uint i=0; i<child.SOG.lengthOfSet; i++){
					write(i, ": ", child.SOG.tracks[i][s].getx(), ", ");
				}
				writeln(")");
			}

			writeln("1");
			foreach(string s, dof; child.g6dofs){
				write(s, " ( ");
				for(uint i=0; i<child.SOG.lengthOfSet; i++){
					write(i, ":", parents[k].SOG.tracks[i][s].getx(), ", ");
				}
				writeln(")");
			}

			writeln("2");
			foreach(string s, dof; child.g6dofs){
				write(s, " ( ");
				for(uint i=0; i<child.SOG.lengthOfSet; i++){
					write(i, ":", parents[l].SOG.tracks[i][s].getx(), ", ");
				}
				writeln(")");
			}
		}
		*/



	}


}
