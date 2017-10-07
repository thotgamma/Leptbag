import std.algorithm;
import std.stdio;
import std.random;

void main(){

	Random rnd = Random(unpredictableSeed);

	int[] neko;

	//prepareCats(neko);

	neko.length = 5;
	foreach(int i, int elem; neko){
		writeln(i);
		writeln(elem);
		elem = i;
	}
	writeln(neko);

}

void prepareCats(ref int[] cats){
	cats.length = 5;
	foreach(int i, elem; cats){
		writeln(i);
		elem = i;
	}
}

