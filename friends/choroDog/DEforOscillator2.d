import std.stdio;
import std.math;
import std.json;
import std.random;
import std.conv;
import std.algorithm;

import japariSDK.japarilib;
import chorodog;
import GA;
import DEforOscillator2;
import Oscillator;
import params;

Random rnd;


chorodog[] evolve(chorodog[] parents, float Cr, float F){


	chorodog[] children;
	children.length = parents.length;
	foreach(int i, ref child; children) child = new chorodog(to!float(i)*5.0f, 0.0f, 0.0f, true);

	auto rnd = Random(unpredictableSeed);


	foreach(int j, child; children){


			uint k, l, m;
			k = uniform(0, to!int(parents.length), rnd);
			l = uniform(0, to!int(parents.length), rnd);
			m = uniform(0, to!int(parents.length), rnd);

			child.gene.friction = parents[k].gene.friction + F * (parents[l].gene.friction - parents[m].gene.friction);
			child.gene.degree = to!int(to!float(parents[k].gene.degree) + F * to!float(parents[l].gene.degree - parents[m].gene.degree) );

			foreach( string s, elem; child.gene.angLimitLower ){

				child.gene.angLimitLower[s] = createVec3(
						parents[k].gene.angLimitLower[s].getx() + F * (parents[l].gene.angLimitLower[s].getx() - parents[m].gene.angLimitLower[s].getx()),
						parents[k].gene.angLimitLower[s].gety() + F * (parents[l].gene.angLimitLower[s].gety() - parents[m].gene.angLimitLower[s].gety()),
						parents[k].gene.angLimitLower[s].getz() + F * (parents[l].gene.angLimitLower[s].getz() - parents[m].gene.angLimitLower[s].getz()));

				child.gene.angLimitUpper[s] = createVec3(
						parents[k].gene.angLimitUpper[s].getx() + F * (parents[l].gene.angLimitUpper[s].getx() - parents[m].gene.angLimitUpper[s].getx()),
						parents[k].gene.angLimitUpper[s].gety() + F * (parents[l].gene.angLimitUpper[s].gety() - parents[m].gene.angLimitUpper[s].gety()),
						parents[k].gene.angLimitUpper[s].getz() + F * (parents[l].gene.angLimitUpper[s].getz() - parents[m].gene.angLimitUpper[s].getz()));

				child.gene.maxForce[s] = createVec3(
						parents[k].gene.maxForce[s].getx() + F * (parents[l].gene.maxForce[s].getx() - parents[m].gene.maxForce[s].getx()),
						parents[k].gene.maxForce[s].gety() + F * (parents[l].gene.maxForce[s].gety() - parents[m].gene.maxForce[s].gety()),
						parents[k].gene.maxForce[s].getz() + F * (parents[l].gene.maxForce[s].getz() - parents[m].gene.maxForce[s].getz()));

				child.gene.maxVelo[s] = createVec3(
						parents[k].gene.maxVelo[s].getx() + F * (parents[l].gene.maxVelo[s].getx() - parents[m].gene.maxVelo[s].getx()),
						parents[k].gene.maxVelo[s].gety() + F * (parents[l].gene.maxVelo[s].gety() - parents[m].gene.maxVelo[s].gety()),
						parents[k].gene.maxVelo[s].getz() + F * (parents[l].gene.maxVelo[s].getz() - parents[m].gene.maxVelo[s].getz()));

				child.gene.oscil.theta[s] = parents[k].gene.oscil.theta[s] + F * (parents[l].gene.oscil.theta[s] - parents[m].gene.oscil.theta[s]);
				child.gene.oscil.phi[s] = parents[k].gene.oscil.phi[s] + F * (parents[l].gene.oscil.phi[s] - parents[m].gene.oscil.phi[s]);
				child.gene.oscil.omegaTheta[s] = parents[k].gene.oscil.omegaTheta[s] + F * (parents[l].gene.oscil.omegaTheta[s] - parents[m].gene.oscil.omegaTheta[s]);
				child.gene.oscil.omegaPhi[s] = parents[k].gene.oscil.omegaPhi[s] + F * (parents[l].gene.oscil.omegaPhi[s] - parents[m].gene.oscil.omegaPhi[s]);

				for(int i=0; i<parents[k].gene.oscil.sinCoeffTheta[s].length; i++) parents[k].gene.oscil.sinCoeffTheta[s][i] = parents[k].gene.oscil.sinCoeffTheta[s][i] + F * (parents[l].gene.oscil.sinCoeffTheta[s][i] + parents[m].gene.oscil.sinCoeffTheta[s][i]);
				for(int i=0; i<parents[k].gene.oscil.cosCoeffTheta[s].length; i++) parents[k].gene.oscil.cosCoeffTheta[s][i] = parents[k].gene.oscil.cosCoeffTheta[s][i] + F * (parents[l].gene.oscil.cosCoeffTheta[s][i] + parents[m].gene.oscil.cosCoeffTheta[s][i]);
				for(int i=0; i<parents[k].gene.oscil.sinCoeffPhi[s].length; i++) parents[k].gene.oscil.sinCoeffPhi[s][i] = parents[k].gene.oscil.sinCoeffPhi[s][i] + F * (parents[l].gene.oscil.sinCoeffPhi[s][i] + parents[m].gene.oscil.sinCoeffPhi[s][i]);
				for(int i=0; i<parents[k].gene.oscil.cosCoeffPhi[s].length; i++) parents[k].gene.oscil.cosCoeffPhi[s][i] = parents[k].gene.oscil.cosCoeffPhi[s][i] + F * (parents[l].gene.oscil.cosCoeffPhi[s][i] + parents[m].gene.oscil.cosCoeffPhi[s][i]);

			}


			foreach(int i, dnas; child.dna ){
				foreach(string s, eachdna; dnas){
					if( Cr > uniform(0.0f, 1.0L, rnd) ){
						eachdna = parents[k].dna[i][s] + F * (parents[l].dna[i][s] - parents[m].dna[i][s]);
						
					}else{
						eachdna = parents[j].dna[i][s];
					}
				}
			}

	}


	return children;
}
