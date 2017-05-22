#include <iostream>
#include <math.h>
#include <deque>

#include "Eigen/Core"

class qNetwork{
	public:
		int hiddenLayer1, hiddenLayer2;
		Eigen::MatrixXf W1,W2,W3;
		Eigen::VectorXf b1,b2,b3;
		//遅延評価のために出力したQのセットの中の最大値，およびそのときの重みとバイアスを遅延区間分保存しておく
		std::deque<Eigen::MatrixXf> input;
		std::deque<Eigen::MatrixXf> qout;
		std::deque<Eigen::MatrixXf> weight[3];
		std::deque<Eigen::VectorXf> bias[3];
		double learningRate = 0.1; //学習率


		qNetwork( int inputSize, int hiddenLayer1, int hiddenLayer2){
			this->hiddenLayer1 = hiddenLayer1;
			this->hiddenLayer2 = hiddenLayer2;
			this->W1 = Eigen::MatrixXf::Random(inputSize, hiddenLayer1);
			this->b1 = Eigen::VectorXf::Random(hiddenLayer1);
			this->W2 = Eigen::MatrixXf::Random(hiddenLayer1, hiddenLayer2);
			this->b2 = Eigen::VectorXf::Random(hiddenLayer2);
			this->W3 = Eigen::MatrixXf::Random(hiddenLayer2, 1);
			this->b3 = Eigen::VectorXf::Random(1);
		}

		Eigen::MatrixXf forward(Eigen::MatrixXf input){

			this->input.push_front(input);

			input = input * this->W1;
			input = input.rowwise() + this->b1.transpose();
			input = input.array().tanh().matrix();
			input = input * this->W2;
			input = input.rowwise() + this->b2.transpose();
			input = input.array().tanh().matrix();
			input = input * this->W3;
			input = input.rowwise() + this->b3.transpose();

			this->qout.push_front(input); //Qは保存しておく
			//重みを保存しておく
			this->weight[0].push_front( this->W1 );
			this->weight[1].push_front( this->W2 );
			this->weight[2].push_front( this->W3 );
			this->bias[0].push_front( this->b1 );
			this->bias[1].push_front( this->b2 );
			this->bias[2].push_front( this->b3 );


			return input;

		}

		Eigen::MatrixXf gradTanh(Eigen::MatrixXf input, Eigen::MatrixXf dout){
			input = 2.0 * input;

			Eigen::MatrixXf plus = input.array().exp() + 1;
			Eigen::MatrixXf minus = input.array().exp() - 1;

			return 2.0*( plus.array().inverse() - minus.array() /* *( ( plus.array()*plus.array() ).cwiseInverse() ) */ ).array() * dout.array();


		}

		//中間ファイルを生成しないようにするために式が長いのでここを読みたかったらchoroから計算グラフをもらってください
		void backward(double error){

			Eigen::MatrixXf dout = Eigen::MatrixXf::Zero(256, 1);
			Eigen::Index maxIndex,maxCol;
			this->qout.back().maxCoeff(&maxIndex, &maxCol);
			dout(maxIndex, 0) = error;


			Eigen::MatrixXf prePara[2];
			prePara[0] = ( this->input.back() * this->weight[0].back() ).rowwise() + this->bias[0].back().transpose();
			prePara[1] = ( prePara[0].array().tanh().matrix() * this->weight[1].back() ).rowwise() + this->bias[1].back().transpose();
			
			Eigen::MatrixXf gradtan = gradTanh(prePara[1], dout * this->weight[2].back().transpose() );
			this->b3 = this->b3 - learningRate * dout.block(maxIndex, 0, 1, 1);
			this->W3 = this->W3 - prePara[1].array().tanh().matrix().transpose() * dout;
			this->b2 = this->b2 - gradtan.block(maxIndex, 0,  1, this->hiddenLayer2).transpose();
			this->W2 = this->W2 - prePara[0].array().tanh().matrix().transpose() * gradtan;
			this->b1 = this->b1 - ( gradTanh( prePara[0], gradtan  * this->weight[1].back().transpose() ) ).block(maxIndex, 0, 1, this->hiddenLayer1).transpose();
			this->W1 = this->W1 - this->input.back().transpose() * gradTanh( prePara[0], gradtan * this->weight[1].back().transpose() );
			//std::cout<<gradTanh( prePara[0], gradtan * this->weight[1].back().transpose() )<<std::endl<<std::endl;

			this->qout.pop_back();
			this->input.pop_back();
			for(int w=0; w<3; w++) this->weight[w].pop_back();
			for(int b=0; b<3; b++) this->bias[b].pop_back();

		}





};
