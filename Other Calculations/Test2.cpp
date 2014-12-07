#include <iostream>
#include <cmath>
#include <iomanip>
#include <fstream>
#include <string>
#include <sstream>
using namespace std;



int main()
{
	ifstream fin("out.txt");
	
	double temp1=0.0;
	double temp2=0.0;
	double dt=0.0;
	
	int counter=0;
	
	fin>>temp1;
	
	double sum=0.0;
	cout<<fixed;
	while(!fin.eof())
	{
		fin>>temp2;
		dt=temp2-temp1;
		temp1=temp2;
		
		counter++;
		sum+=dt;
		cout<<temp2<<endl;
		
	}
	double average=sum/counter;
	cout<<"Sum: "<<sum<<endl;
	cout<<"Count: "<<counter<<endl;
	cout<<"Avg: "<<average<<endl;
	cout<<"Freq: "<<1.0/average<<endl;
	
	return 0;
}

