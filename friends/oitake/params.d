import core.runtime;
import std.stdio;
import std.math;
import std.json;
import std.random;
import std.conv;
import std.algorithm;

import japariSDK.japarilib;
import Oscillator;


struct agentBodyParameter{
	elementManager[string] partsGenerator;
	partParam[string] partParams; //身体パーツのパラメータ
	hingeParam[string] hingeParams; //ヒンジのパラメータ
	g6dofParam[string] g6dofParams; //g6dofのパラメータ
}

//身体パーツパラメータ
struct partParam{

	vertexManager vertices;
	vec3 position;
	vec3 scale;
	quat rotation;
	float mass; //総体重に対する百分率
	float friction; //摩擦係数

}



//ヒンジパラメータ
struct hingeParam{

	string name;
	vec3 position;
	vec3 axis1;
	vec3 axis2;
	string object1Name;
	string object2Name;
	vec3 object1Position;
	vec3 object2Position;
	bool enabled;
	bool useLimit;
	float limitLower;
	float limitUpper;


}

//g6dofパラメータ
struct g6dofParam{

	string name;
	bool enabled;
	vec3 position;
	quat rotation;
	string object1Name;
	string object2Name;
	vec3 object1Position;
	vec3 object2Position;
	bool[3] useAngLimit; //(x, y, z) : ( 0, 1, 2 )
	vec3 angLimitLower;
	vec3 angLimitUpper;
	bool[3] useLinLimit;
	vec3 linLimitLower;
	vec3 linLimitUpper;

}

//遺伝させるパラメータ

struct serialOrderGene{
	const uint lengthOfSet = 20;
	vec3[string][lengthOfSet] tracks;

	void init(){}

	void init(string s, vec3 lowerLimit, vec3 upperLimit){
		auto rnd = Random(unpredictableSeed);
		for(int i=0; i<lengthOfSet; i++){

			float x, y, z;
			if(lowerLimit.getx()<upperLimit.getx()){
				x = uniform(lowerLimit.getx(), upperLimit.getx(), rnd);
			}else{
				x = 0.0f;
			}

			if(lowerLimit.gety()<upperLimit.gety()){
				y = uniform(lowerLimit.gety(), upperLimit.gety(), rnd);
			}else{
				y = 0.0f;
			}

			if(lowerLimit.getz()<upperLimit.getz()){
				z = uniform(lowerLimit.getz(), upperLimit.getz(), rnd);
			}else{
				z = 0.0f;
			}

			tracks[i][s] = createVec3(x, y, z);
			//write(s, ":", i, "(", tracks[i][s].getx(), ", ", tracks[i][s].gety(), ")");
		}
	}

	void init(int i, string s, vec3 lowerLimit, vec3 upperLimit){
		writeln("buma");
		auto rnd = Random(unpredictableSeed);

			float x, y, z;
			if(lowerLimit.getx()<upperLimit.getx()){
				x = uniform(lowerLimit.getx(), upperLimit.getx(), rnd);
			}else{
				x = 0.0f;
			}

			if(lowerLimit.gety()<upperLimit.gety()){
				y = uniform(lowerLimit.gety(), upperLimit.gety(), rnd);
			}else{
				y = 0.0f;
			}

			if(lowerLimit.getz()<upperLimit.getz()){
				z = uniform(lowerLimit.getz(), upperLimit.getz(), rnd);
			}else{
				z = 0.0f;
			}

			writeln(x, ", ", y, ", ", z);

			tracks[i][s] = createVec3(x, y, z);

	}


	void copytracks(serialOrderGene u){
		foreach(int i, elem1; this.tracks){
			foreach(string s, elem2; elem1){
				this.tracks[i][s] = createVec3( u.tracks[i][s].getx(), u.tracks[i][s].gety(), u.tracks[i][s].getz() );
			}
		}
	}

	void copytracks(serialOrderGene u,int i,string s){
		this.tracks[i][s] = createVec3( u.tracks[i][s].getx(), u.tracks[i][s].gety(), u.tracks[i][s].getz() );
	}


}


struct oscillator2Gene{

	vec3[string] angLimitLower;
	vec3[string] angLimitUpper;
	float friction;
	vec3[string] maxForce; //最大出力．いくらmaxVeloを大きくしてもこれ以上の力では駆動しない．
	vec3[string] maxVelo; //g6dofを動かす最高速
	oscillator2 oscil; //振動子モデル．1個体に1つ．
	int degree; //振動子モデルの近似精度(sin(nx), cos(nx)のn)

	//関節間で共通するパラメータの初期化
	void init(){
		degree = 5;
		oscil = new oscillator2(degree);
		auto rnd = Random(unpredictableSeed);
		friction = 2.0f;//uniform(0.0f, 5.0f, rnd);
	}


	//各関節で異なるパラメータの初期化
	//blenderで定義した関節制限角度を用いない場合
	void init(string s){
		auto rnd = Random(unpredictableSeed);

		maxForce[s] = createVec3( uniform(0.0f, 10.0f, rnd), uniform(0.0f, 10.0f, rnd), 0.0f );
		maxVelo[s] = createVec3( uniform(0.0f, 10.0f, rnd), uniform(0.0f, 10.0f, rnd), uniform(0.0f, 10.0f, rnd) );

		oscil.init(s);

		angLimitUpper[s] = createVec3( uniform(0.0f, 1.57f, rnd), uniform(0.0f, 1.57f, rnd), 0.0f );
		angLimitLower[s] = createVec3( uniform(-1.57f, 0.0f, rnd), uniform(-1.57f, 0.0f, rnd), 0.0f );

	}

	//各関節で異なるパラメータの初期化
	//関節角度をblenderから読込む場合
	void init(string s, g6dofParam dofParam){

		auto rnd = Random(unpredictableSeed);
		maxForce[s] = createVec3( uniform(0.0f, 10.0f, rnd), uniform(0.0f, 10.0f, rnd), 0.0f );
		maxVelo[s] = createVec3( uniform(0.0f, 10.0f, rnd), uniform(0.0f, 10.0f, rnd), uniform(0.0f, 10.0f, rnd) );

		oscil.init(s);

		angLimitLower[s] = dofParam.angLimitLower;
		angLimitUpper[s] = dofParam.angLimitUpper;

	}

	void rehash(){
		angLimitLower.rehash;
		angLimitUpper.rehash;
		maxForce.rehash;
		maxVelo.rehash;
	}

	//表示関数．oscilのtoString()は実装してない．
	void toString(){

		write("angLimitLower : ");
		foreach(string s, elem; angLimitLower){
			write("[ \"", s, "\": ", elem.getx(), ", ", elem.gety(), ", ", elem.getz(), " ], ");
		}
		writeln("");

		write("angLimitUpper : ");
		foreach(string s, elem; angLimitUpper){
			write("[ \"", s, "\": ", elem.getx(), ", ", elem.gety(), ", ", elem.getz(), " ], ");
		}
		writeln("");

		writeln("friction = ", friction);

		write("maxForce : ");
		foreach(string s, elem; maxForce){
			write("[ \"", s, "\": ", elem.getx(), ", ", elem.gety(), ", ", elem.getz(), " ], ");
		}
		writeln("");

		write("maxVelo : ");
		foreach(string s, elem; maxVelo){
			write("[ \"", s, "\": ", elem.getx(), ", ", elem.gety(), ", ", elem.getz(), " ], ");
		}
		writeln("");

		//oscil.toString();
		writeln("degree = ", degree);


	}


}
