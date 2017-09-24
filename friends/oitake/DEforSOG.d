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

void evolveSOG(agent[] children, agent[] parents, float Cr, float F, int[] bests){

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

					child.SOG.tracks[i][s] = parents[j].SOG.tracks[i][s];

				}

			}
		}

	}


}
