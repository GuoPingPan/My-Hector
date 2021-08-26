#ifndef PARAMTER_READER_H_
#define PARAMTER_READER_H_
#include <iostream>
#include <string>
#include <map>
#include <fstream>
#include <typeinfo>

using namespace std;
class ParamterReader {
public:
    ParamterReader() {};

    ParamterReader(string filename) {
        ifstream fin(filename.c_str());
        if (!fin) {
            cerr << "the paramter file is not exist." << endl;
            return;
        }
        cout<<"reading from "<< filename.c_str()<<endl;
        while (!fin.eof()) {
            string str;
            getline(fin, str);
            if (str[0] == '#')
                continue;
            int pos = str.find('=');
            if (pos == -1)
                continue;
            string key = str.substr(0, pos);
            key = key.erase(0,key.find_first_not_of(" "));
            key = key.erase(key.find_last_not_of(" ")+1);
            string value = str.substr(pos + 1, str.length());
            value = value.erase(0,value.find_first_not_of(" "));
            value = value.erase(value.find_last_not_of(" ")+1);
            data[key] = value;
            cout<<key<<" : "<<value<<endl;
            if (!fin.good())
                break;
        }
    }

    ~ParamterReader(){
        data.clear();
    };

    template<typename valuetype>
    valuetype getData(string key, valuetype default_value) {
        map<string, string>::iterator it;
        it = data.find(key);
        if (it == data.end()) {
            cout << "the data does not exist,use the default to fill it." << endl;
            cout << key <<endl;
            return default_value;
        }
        if (typeid(valuetype) == typeid(double))
            return value_cast_double(it->second);
        else if (typeid(valuetype) == typeid(float))
            return value_cast_float(it->second);
        else if (typeid(valuetype) == typeid(int))
            return value_cast_int(it->second);
        else if (typeid(valuetype) == typeid(bool))
            return value_cast_bool(it->second);

    }

    inline float value_cast_float(string value) { return atof(value.c_str()); }
    inline double value_cast_double(string value) { return atof(value.c_str()); }
    inline int value_cast_int(string value) { return atoi(value.c_str()); }
    inline bool value_cast_bool(string value) { return value == "true" ? true : false; }

    string getString(string key, string default_value) {
        map<string, string>::iterator it;
        it = data.find(key);
        if (it != data.end()) {
            return it->second;
        }
        else
            return default_value;
    }

public:
    map<string,string> data;
};

#endif //PARAMTER_READER_H_