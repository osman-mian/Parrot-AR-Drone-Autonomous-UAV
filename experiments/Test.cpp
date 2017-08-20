#include <iostream>
#include <cmath>
#include <iomanip>
#include <fstream>
#include <string>
#include <sstream>
using namespace std;



int main()
{
	ifstream fin("input.txt");
	ofstream fout("out.txt");
	
	stringstream strin;
	string x;
	double y,b;
	double sum=0.0;
	int counter=0;
	fout<<fixed<<setprecision(9);
	
	while(!fin.eof())
	//for(int i=0;i<10;i++)
	{
		fin>>x;
		strin<<x;
		strin>>y;
		strin.clear();
		
		fin>>x;
		strin<<x;
		strin>>b;
		strin.clear();

		sum+=(y+ b/pow(10,9));
		fout<<(y+ b/pow(10,9))<<endl;
		cout<<(y+ b/pow(10,9))<<endl;
		
		
	}
	
	
	return 0;
}

