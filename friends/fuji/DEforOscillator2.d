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


//Differential Evolution
void evolve(agent[] children, agent[] parents, float Cr, float F){


	auto rnd = Random(unpredictableSeed);


	//交雑
	foreach(int j, child; children){


		//ランダムに3個体を選ぶ．
		//そのパラメータ全体(Oscillator2Gene)をベクトルとしてみる．
		//3つのうち1つのベクトルに，ほか2つのベクトルの差分*Fを加えたベクトルを生成する
		//確率Crでそのベクトルの各成分を採用する
		uint k, l, m;
		k = uniform(0, to!int(parents.length), rnd);
		l = uniform(0, to!int(parents.length), rnd);
		m = uniform(0, to!int(parents.length), rnd);

		//以下，ベクトル生成

		if(Cr < uniform(0.0f, 1.0f, rnd)) child.gene.friction = parents[k].gene.friction + F * (parents[l].gene.friction - parents[m].gene.friction);
		else child.gene.friction = parents[j].gene.friction;

		if(Cr < uniform(0.0f, 1.0f, rnd)) child.gene.degree = to!int(to!float(parents[k].gene.degree) + F * to!float(parents[l].gene.degree - parents[m].gene.degree) );
		else child.gene.degree = parents[j].gene.degree;

		foreach( string s, elem; child.gene.angLimitLower ){

			if(Cr < uniform(0.0f, 1.0f, rnd)) child.gene.angLimitLower[s] = createVec3(
					parents[k].gene.angLimitLower[s].getx() + F * (parents[l].gene.angLimitLower[s].getx() - parents[m].gene.angLimitLower[s].getx()),
					parents[k].gene.angLimitLower[s].gety() + F * (parents[l].gene.angLimitLower[s].gety() - parents[m].gene.angLimitLower[s].gety()),
					parents[k].gene.angLimitLower[s].getz() + F * (parents[l].gene.angLimitLower[s].getz() - parents[m].gene.angLimitLower[s].getz()));
			else child.gene.angLimitLower[s] = parents[j].gene.angLimitLower[s];

			if(Cr < uniform(0.0f, 1.0f, rnd)) child.gene.angLimitUpper[s] = createVec3(
					parents[k].gene.angLimitUpper[s].getx() + F * (parents[l].gene.angLimitUpper[s].getx() - parents[m].gene.angLimitUpper[s].getx()),
					parents[k].gene.angLimitUpper[s].gety() + F * (parents[l].gene.angLimitUpper[s].gety() - parents[m].gene.angLimitUpper[s].gety()),
					parents[k].gene.angLimitUpper[s].getz() + F * (parents[l].gene.angLimitUpper[s].getz() - parents[m].gene.angLimitUpper[s].getz()));
			else child.gene.angLimitUpper[s] = parents[j].gene.angLimitUpper[s];

			if(Cr < uniform(0.0f, 1.0f, rnd)) child.gene.maxForce[s] = createVec3(
					parents[k].gene.maxForce[s].getx() + F * (parents[l].gene.maxForce[s].getx() - parents[m].gene.maxForce[s].getx()),
					parents[k].gene.maxForce[s].gety() + F * (parents[l].gene.maxForce[s].gety() - parents[m].gene.maxForce[s].gety()),
					parents[k].gene.maxForce[s].getz() + F * (parents[l].gene.maxForce[s].getz() - parents[m].gene.maxForce[s].getz()));
			else child.gene.maxForce[s] = parents[j].gene.maxForce[s];

			if(Cr < uniform(0.0f, 1.0f, rnd)) child.gene.maxVelo[s] = createVec3(
					parents[k].gene.maxVelo[s].getx() + F * (parents[l].gene.maxVelo[s].getx() - parents[m].gene.maxVelo[s].getx()),
					parents[k].gene.maxVelo[s].gety() + F * (parents[l].gene.maxVelo[s].gety() - parents[m].gene.maxVelo[s].gety()),
					parents[k].gene.maxVelo[s].getz() + F * (parents[l].gene.maxVelo[s].getz() - parents[m].gene.maxVelo[s].getz()));
			else child.gene.maxVelo[s] = parents[j].gene.maxVelo[s];
		}

		foreach(string s, th; child.gene.oscil.theta){

			if(Cr < uniform(0.0f, 1.0f, rnd)) child.gene.oscil.omegaTheta[s] = parents[k].gene.oscil.omegaTheta[s] + F * (parents[l].gene.oscil.omegaTheta[s] - parents[m].gene.oscil.omegaTheta[s]);
			else child.gene.oscil.omegaTheta[s] = parents[j].gene.oscil.omegaTheta[s];

			if(Cr < uniform(0.0f, 1.0f, rnd)) child.gene.oscil.omegaPhi[s] = parents[k].gene.oscil.omegaPhi[s] + F * (parents[l].gene.oscil.omegaPhi[s] - parents[m].gene.oscil.omegaPhi[s]);
			else child.gene.oscil.omegaPhi[s] = parents[j].gene.oscil.omegaPhi[s];

			for(int i=0; i<parents[k].gene.oscil.sinCoeffTheta[s].length; i++){

				if(Cr < uniform(0.0f, 1.0f, rnd)) 
					child.gene.oscil.sinCoeffTheta[s][i] = parents[k].gene.oscil.sinCoeffTheta[s][i] + F * (parents[l].gene.oscil.sinCoeffTheta[s][i] + parents[m].gene.oscil.sinCoeffTheta[s][i]);
				else child.gene.oscil.sinCoeffTheta[s][i] = parents[j].gene.oscil.sinCoeffTheta[s][i];

			}

			for(int i=0; i<parents[k].gene.oscil.cosCoeffTheta[s].length; i++){
				if(Cr < uniform(0.0f, 1.0f, rnd)) 
					child.gene.oscil.cosCoeffTheta[s][i] = parents[k].gene.oscil.cosCoeffTheta[s][i] + F * (parents[l].gene.oscil.cosCoeffTheta[s][i] + parents[m].gene.oscil.cosCoeffTheta[s][i]);
				else child.gene.oscil.cosCoeffTheta[s][i] = parents[j].gene.oscil.cosCoeffTheta[s][i];
			}

			for(int i=0; i<parents[k].gene.oscil.sinCoeffPhi[s].length; i++){
				if(Cr < uniform(0.0f, 1.0f, rnd)) 
					child.gene.oscil.sinCoeffPhi[s][i] = parents[k].gene.oscil.sinCoeffPhi[s][i] + F * (parents[l].gene.oscil.sinCoeffPhi[s][i] + parents[m].gene.oscil.sinCoeffPhi[s][i]);
				else child.gene.oscil.sinCoeffPhi[s][i] = parents[j].gene.oscil.sinCoeffPhi[s][i];
			}

			for(int i=0; i<parents[k].gene.oscil.cosCoeffPhi[s].length; i++){
				if(Cr < uniform(0.0f, 1.0f, rnd)) 
					child.gene.oscil.cosCoeffPhi[s][i] = parents[k].gene.oscil.cosCoeffPhi[s][i] + F * (parents[l].gene.oscil.cosCoeffPhi[s][i] + parents[m].gene.oscil.cosCoeffPhi[s][i]);
				else child.gene.oscil.cosCoeffPhi[s][i] = parents[j].gene.oscil.cosCoeffPhi[s][i];
			}

		}



	}


}



void evolveBest(agent[] children, agent[] parents, float Cr, float F, int[] bests){


	auto rnd = Random(unpredictableSeed);


	//agentのgeneに含まれるパラメータを交雑
	foreach(int j, child; children){


		//ランダムに3個体を選ぶ．
		//そのパラメータ全体(Oscillator2Gene)をベクトルとしてみる．
		//3つのうち1つのベクトルに，ほか2つのベクトルの差分*Fを加えたベクトルを生成する
		//確率Crでそのベクトルの各成分を採用する
		uint k, l, m;
		k = bests[0];
		l = bests[1];
		m = bests[2];

		//以下，ベクトル生成

		if(Cr < uniform(0.0f, 1.0f, rnd)) child.gene.friction = parents[k].gene.friction + F * (parents[l].gene.friction - parents[m].gene.friction);
		else child.gene.friction = parents[j].gene.friction;

		if(Cr < uniform(0.0f, 1.0f, rnd)) child.gene.degree = to!int(to!float(parents[k].gene.degree) + F * to!float(parents[l].gene.degree - parents[m].gene.degree) );
		else child.gene.degree = parents[j].gene.degree;

		foreach( string s, elem; child.gene.angLimitLower ){

			if(Cr < uniform(0.0f, 1.0f, rnd)) child.gene.angLimitLower[s] = createVec3(
					parents[k].gene.angLimitLower[s].getx() + F * (parents[l].gene.angLimitLower[s].getx() - parents[m].gene.angLimitLower[s].getx()),
					parents[k].gene.angLimitLower[s].gety() + F * (parents[l].gene.angLimitLower[s].gety() - parents[m].gene.angLimitLower[s].gety()),
					parents[k].gene.angLimitLower[s].getz() + F * (parents[l].gene.angLimitLower[s].getz() - parents[m].gene.angLimitLower[s].getz()));
			else child.gene.angLimitLower[s] = parents[j].gene.angLimitLower[s];

			if(Cr < uniform(0.0f, 1.0f, rnd)) child.gene.angLimitUpper[s] = createVec3(
					parents[k].gene.angLimitUpper[s].getx() + F * (parents[l].gene.angLimitUpper[s].getx() - parents[m].gene.angLimitUpper[s].getx()),
					parents[k].gene.angLimitUpper[s].gety() + F * (parents[l].gene.angLimitUpper[s].gety() - parents[m].gene.angLimitUpper[s].gety()),
					parents[k].gene.angLimitUpper[s].getz() + F * (parents[l].gene.angLimitUpper[s].getz() - parents[m].gene.angLimitUpper[s].getz()));
			else child.gene.angLimitUpper[s] = parents[j].gene.angLimitUpper[s];

			if(Cr < uniform(0.0f, 1.0f, rnd)) child.gene.maxForce[s] = createVec3(
					parents[k].gene.maxForce[s].getx() + F * (parents[l].gene.maxForce[s].getx() - parents[m].gene.maxForce[s].getx()),
					parents[k].gene.maxForce[s].gety() + F * (parents[l].gene.maxForce[s].gety() - parents[m].gene.maxForce[s].gety()),
					parents[k].gene.maxForce[s].getz() + F * (parents[l].gene.maxForce[s].getz() - parents[m].gene.maxForce[s].getz()));
			else child.gene.maxForce[s] = parents[j].gene.maxForce[s];

			if(Cr < uniform(0.0f, 1.0f, rnd)) child.gene.maxVelo[s] = createVec3(
					parents[k].gene.maxVelo[s].getx() + F * (parents[l].gene.maxVelo[s].getx() - parents[m].gene.maxVelo[s].getx()),
					parents[k].gene.maxVelo[s].gety() + F * (parents[l].gene.maxVelo[s].gety() - parents[m].gene.maxVelo[s].gety()),
					parents[k].gene.maxVelo[s].getz() + F * (parents[l].gene.maxVelo[s].getz() - parents[m].gene.maxVelo[s].getz()));
			else child.gene.maxVelo[s] = parents[j].gene.maxVelo[s];
		}

		foreach(string s, th; child.gene.oscil.theta){

			if(Cr < uniform(0.0f, 1.0f, rnd)) child.gene.oscil.omegaTheta[s] = parents[k].gene.oscil.omegaTheta[s] + F * (parents[l].gene.oscil.omegaTheta[s] - parents[m].gene.oscil.omegaTheta[s]);
			else child.gene.oscil.omegaTheta[s] = parents[j].gene.oscil.omegaTheta[s];

			if(Cr < uniform(0.0f, 1.0f, rnd)) child.gene.oscil.omegaPhi[s] = parents[k].gene.oscil.omegaPhi[s] + F * (parents[l].gene.oscil.omegaPhi[s] - parents[m].gene.oscil.omegaPhi[s]);
			else child.gene.oscil.omegaPhi[s] = parents[j].gene.oscil.omegaPhi[s];

			for(int i=0; i<parents[k].gene.oscil.sinCoeffTheta[s].length; i++){

				if(Cr < uniform(0.0f, 1.0f, rnd)) 
					child.gene.oscil.sinCoeffTheta[s][i] = parents[k].gene.oscil.sinCoeffTheta[s][i] + F * (parents[l].gene.oscil.sinCoeffTheta[s][i] + parents[m].gene.oscil.sinCoeffTheta[s][i]);
				else child.gene.oscil.sinCoeffTheta[s][i] = parents[j].gene.oscil.sinCoeffTheta[s][i];

			}

			for(int i=0; i<parents[k].gene.oscil.cosCoeffTheta[s].length; i++){
				if(Cr < uniform(0.0f, 1.0f, rnd)) 
					child.gene.oscil.cosCoeffTheta[s][i] = parents[k].gene.oscil.cosCoeffTheta[s][i] + F * (parents[l].gene.oscil.cosCoeffTheta[s][i] + parents[m].gene.oscil.cosCoeffTheta[s][i]);
				else child.gene.oscil.cosCoeffTheta[s][i] = parents[j].gene.oscil.cosCoeffTheta[s][i];
			}

			for(int i=0; i<parents[k].gene.oscil.sinCoeffPhi[s].length; i++){
				if(Cr < uniform(0.0f, 1.0f, rnd)) 
					child.gene.oscil.sinCoeffPhi[s][i] = parents[k].gene.oscil.sinCoeffPhi[s][i] + F * (parents[l].gene.oscil.sinCoeffPhi[s][i] + parents[m].gene.oscil.sinCoeffPhi[s][i]);
				else child.gene.oscil.sinCoeffPhi[s][i] = parents[j].gene.oscil.sinCoeffPhi[s][i];
			}

			for(int i=0; i<parents[k].gene.oscil.cosCoeffPhi[s].length; i++){
				if(Cr < uniform(0.0f, 1.0f, rnd)) 
					child.gene.oscil.cosCoeffPhi[s][i] = parents[k].gene.oscil.cosCoeffPhi[s][i] + F * (parents[l].gene.oscil.cosCoeffPhi[s][i] + parents[m].gene.oscil.cosCoeffPhi[s][i]);
				else child.gene.oscil.cosCoeffPhi[s][i] = parents[j].gene.oscil.cosCoeffPhi[s][i];
			}

		}



	}


}
