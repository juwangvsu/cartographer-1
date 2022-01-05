#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <iostream>
#include <fstream>
#include <map>
#include <boost/algorithm/string/trim.hpp>
#include <string.h>
#include <vector>
using namespace std;
template<class T> struct Point{
    template <typename NewType> Point<NewType> cast() const{
        return Point<NewType>();
    }
};
struct tt{
	int x;
	int y;
};
main()
{
    Point<float> p1;
    Point<double> p2;
    tt t1{1,2};
    tt t2(1,2);
    std::cout<<"struct tt: " <<t2.x<<"\n";
    std::vector<int> v = { 7, 5, 16, 8 };
    std::vector<int> intvec1;
    std::cout <<"intvec1.size(): "<< intvec1.size();
    intvec1.push_back(3);
    std::cout <<"\nintvec1.front(): "<< intvec1.front()<<"\n";

    //std::cout<<p1 <<" "<<p2 <<"\n";
    p2 = p1.cast<double>();
	std::array<int,3> a1{1,2,3};
	std::cout<<a1.data()[2];
	std::cout<<a1.at(2);

    std::cout<<"hello";
    std::string filename="testopt2.txt";

    std::cout<<NULL<<"----\n";
std::ifstream stream(filename.c_str());
//    std::ifstream stream("testopt2.txt");
    std::string myoptstr= std::string((std::istreambuf_iterator<char>(stream)),
                     std::istreambuf_iterator<char>());
    std::cout<< myoptstr;
map<string, string> mymap;
char *token;
token = strtok(const_cast<char*>(myoptstr.c_str()), "\n");
    while (token != NULL) {
        string s(token);
        size_t pos = s.find(":");
        //mymap[s.substr(0, pos)] = boost::algorithm::trim(s);
	string svalue = s.substr(pos + 1, string::npos);
	boost::algorithm::trim(svalue);
        mymap[s.substr(0, pos)] = svalue;
	//boost::algorithm::trim(s.substr(pos + 1, string::npos));
        token = strtok(NULL, "\n");

	if (s.substr(0, pos)=="tst"){
	std::cout<<"test tst"<<std::endl;
		char * token2;
		//std::cout<<svalue<<std::endl;
		token2 = strtok(const_cast<char*>(svalue.c_str()), ",");
		while (token2 != NULL) {
			string ss(token2);
			std::cout<<token2 <<" ";
        		token2 = strtok(NULL, ",");
		}

	}
    }

    std::cout<<"\n*****************\ncout mymap:\n";
    for (auto keyval : mymap)
        cout << keyval.first << ":" << keyval.second << endl;

}
