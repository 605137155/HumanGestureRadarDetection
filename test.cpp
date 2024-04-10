#include <vector>
#include <QDebug>
using namespace std;
vector<int> nums = { 1,5,7,6,8,4,3,2,1,4,5,9,6,3,1 };


int _main(int argc, char* argv[]) {

	std::vector<std::vector<float>> matrix = { {-1.08930402f,-0.00126659f,-0.05982419f},
					{0.04921241f,1.10069102f,-0.24182825f},
					{0.06662979f,0.21614036f,0.95915324f} };
	qDebug()<< matrix[0][0]<< matrix[0][1]<< matrix[0][2];
	return 0;

}




