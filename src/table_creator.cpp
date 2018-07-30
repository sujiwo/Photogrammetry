#include <iostream>
#include <fstream>
#include <boost/tokenizer.hpp>
#include <string>
#include <vector>
#include <set>
#include <exception>
#include <cstdint>


typedef std::vector<std::vector<std::string>>
	StringTable;
typedef std::vector<std::pair<uint64_t,int>>
	OxfordTimestamp;


StringTable create_table(const std::string &path, const std::set<int> &usingColumns=std::set<int>())
{
	std::ifstream fd;
	fd.open(path);
	if (fd.good()==false)
		throw std::runtime_error("Unable to open file "+path);

	StringTable strTbl;
	bool gotFirstRow = false;

	while (fd.eof() == false) {
		std::string curLine;
		std::getline(fd, curLine);
		if (curLine.size()==0)
			break;

		boost::tokenizer<> stok(curLine);
		std::vector<std::string> vecline;
		for (auto s: stok) {
			vecline.push_back(s);
		}

		if (usingColumns.size()==0)
			strTbl.push_back(vecline);

		else {
			std::vector<std::string> vl;
			for (auto &c: usingColumns) {
				vl.push_back(vecline.at(c));
			}
			strTbl.push_back(vl);
		}

		if (gotFirstRow==false)
			gotFirstRow = true;
	}

	return strTbl;
}


std::vector<std::pair<unsigned int,int>>
make_oxford_timestamp(const std::string &fpath)
{
	std::vector<std::pair<unsigned int,int>> TS;
	StringTable tsTable = create_table(fpath, std::set<int>({0,1}));
	TS.resize(tsTable.size());

	for (int r=0; r<tsTable.size(); r++) {
		uint64_t t = std::stoul(tsTable[r][0]);
		int x = std::stoi(tsTable[r][1]);
		TS[r] = std::make_pair(t,x);
	}

	return TS;
}


int main (int argc, char *argv[])
{
	make_oxford_timestamp(argv[1]);
}
