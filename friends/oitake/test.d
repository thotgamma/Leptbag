import std.algorithm;
import std.stdio;
import std.random;

void main(){

	Random rnd = Random(unpredictableSeed);

	float[2][3] neko;
	
	for(int i=0; i<2; i++){
		for(int j=0; j<3; j++){
			neko[j][i] = uniform(0.0, 1.0, rnd);
		}
	}
	writeln(neko);

}
