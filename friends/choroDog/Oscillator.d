import std.stdio;
import std.math;
import std.random;
import std.conv;
import std.algorithm;
import std.string;



//2自由度系の振動子モデル
class oscillator2{

	public:
		float[string] theta;
		float[string] phi;
		float[string] omegaTheta;
		float[string] omegaPhi;
		float[][string] sinCoeffTheta;
		float[][string] cosCoeffTheta;
		float[][string] sinCoeffPhi;
		float[][string] cosCoeffPhi;
		float degree;


		this(float deg){
			degree = deg;
		}

		void init(string name){
			Random rnd;
			Random(unpredictableSeed);

			for(int i=0; i<degree; i++){
				sinCoeffTheta[name] ~= uniform(-1.0f, 1.0f, rnd);
				cosCoeffTheta[name] ~= uniform(-1.0f, 1.0f, rnd);
				sinCoeffPhi[name] ~= uniform(-1.0f, 1.0f, rnd);
				cosCoeffPhi[name] ~= uniform(-1.0f, 1.0f, rnd);
			}

			omegaTheta[name] = uniform(-1.0f, 1.0f, rnd);
			omegaPhi[name] = uniform(-1.0f, 1.0f, rnd);
		}

		void rehash(){
			theta = theta.rehash;
			phi = phi.rehash;
			sinCoeffTheta = sinCoeffTheta.rehash;
			cosCoeffTheta = cosCoeffTheta.rehash;
			sinCoeffPhi = sinCoeffPhi.rehash;
			cosCoeffPhi = cosCoeffPhi.rehash;
		}

		void setTheta(string name, float th){
			theta[name] = th;
		}

		void setPhi(string name, float ph){
			phi[name] = ph;
		}


		float[string] calculateDeltaTheta(){

			float[string] deltaTheta;

			foreach(string s, th; theta) deltaTheta[s] = omegaTheta[s];

			foreach(string s, me; theta){
				for(int i=0; i<degree; i++){
					foreach(other; theta){
						deltaTheta[s] += sinCoeffTheta[s][i] * sin( i*(other-me) );
						deltaTheta[s] += cosCoeffTheta[s][i] * cos( i*(other-me) );
					}
				}
			}

			return deltaTheta;
		}


		float[string] calculateDeltaPhi(){

			float[string] deltaPhi;

			foreach(string s, ph; phi) deltaPhi[s] = omegaPhi[s];

			foreach(string s, me; phi){
				for(int i=0; i<degree; i++){
					foreach(other; theta){
						deltaPhi[s] += sinCoeffPhi[s][i] * sin( i*(other-me) );
						deltaPhi[s] += cosCoeffPhi[s][i] * cos( i*(other-me) );
					}
				}
			}

			return deltaPhi;
		}

}
