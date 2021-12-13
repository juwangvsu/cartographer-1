#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <iostream>
#include <fstream>
#include <map>
#include <boost/algorithm/string/trim.hpp>
#include <string.h>
using namespace std;
main()
{
    std::cout<<"hello";
    std::string filename="testopt2.txt";

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
    }

    for (auto keyval : mymap)
        cout << keyval.first << ":" << keyval.second << endl;

}
