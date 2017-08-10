import std.stdio;
import std.math;
import std.random;
import std.conv;
import std.algorithm;
import std.string;


//2自由度系の振動子モデル
class oscillator2{

	public:
		float[string] theta; //振動子の現在の位相
		float[string] phi;
		float[string] omegaTheta; //振動子の基本駆動力
		float[string] omegaPhi;
		float[][string] sinCoeffTheta; //sin, cosの各項に対する係数
		float[][string] cosCoeffTheta;
		float[][string] sinCoeffPhi;
		float[][string] cosCoeffPhi;
		int degree; //関数近似精度．sin, cosを何次の項まで計算するか．


		this(int deg){
			degree = deg;
		}

		void init(string name){
			Random rnd;
			Random(unpredictableSeed);

			//初期値はランダム．分布はあとで変える．
			for(int i=0; i<degree; i++){
				sinCoeffTheta[name] ~= uniform(-10.0f, 10.0f, rnd);
				cosCoeffTheta[name] ~= uniform(-10.0f, 10.0f, rnd);
				sinCoeffPhi[name] ~= uniform(-10.0f, 10.0f, rnd);
				cosCoeffPhi[name] ~= uniform(-10.0f, 10.0f, rnd);
			}

			omegaTheta[name] = uniform(-10.0f, 10.0f, rnd);
			omegaPhi[name] = uniform(-10.0f, 10.0f, rnd);
		}


		//rehashすると最適化されるらしい．使いたかっただけ．
		void rehash(){
			theta = theta.rehash;
			phi = phi.rehash;
			omegaTheta = omegaTheta.rehash;
			omegaPhi = omegaPhi.rehash;
			sinCoeffTheta = sinCoeffTheta.rehash;
			cosCoeffTheta = cosCoeffTheta.rehash;
			sinCoeffPhi = sinCoeffPhi.rehash;
			cosCoeffPhi = cosCoeffPhi.rehash;
		}

		//現在角度を入力
		void setTheta(string name, float th){
			theta[name] = th;
		}

		void setPhi(string name, float ph){
			phi[name] = ph;
		}


		//このdeltaThetaは必ずしも角度の微分を意味しない．単なる関数近似器として使う．
		float[string] calculateDeltaTheta(){

			float[string] deltaTheta;

			foreach(string s, th; theta) deltaTheta[s] = omegaTheta[s];

			/+
			foreach(string s, me; theta){
				for(int i=1; i<=degree; i++){
					foreach(other; theta){
						deltaTheta[s] += sinCoeffTheta[s][i-1] * sin( i*(other-me) );
						deltaTheta[s] += cosCoeffTheta[s][i-1] * cos( i*(other-me) );
					}
				}
			}

			deltaTheta.rehash;
			+/

			return deltaTheta;
		}


		//このdeltaPhiは必ずしも角度の微分を意味しない．単なる関数近似器として使う．
		float[string] calculateDeltaPhi(){

			float[string] deltaPhi;

			foreach(string s, ph; phi) deltaPhi[s] = omegaPhi[s];

			foreach(string s, me; phi){
				for(int i=1; i<=degree; i++){
					foreach(other; theta){
						deltaPhi[s] += sinCoeffPhi[s][i-1] * sin( i*(other-me) );
						deltaPhi[s] += cosCoeffPhi[s][i-1] * cos( i*(other-me) );
					}
				}
			}

			deltaPhi.rehash;

			return deltaPhi;
		}

}
