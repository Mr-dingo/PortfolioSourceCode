#pragma once

#include <Editor/BasicBody/BasicBody.h>
// Implemented by Luis Lee <Begin>
namespace dcsbodyspace {

class DCBODYDLL SetBody : public BasicBody {

public:
	SetBody();
	~SetBody();	
	
	bool load(const std::wstring&);
	void update();
	bool isSetBody() { return true; }
	bool isInterpolated(){return interpolation;}
	bool isCalculated(){return calculateHF;}
private:
	bool interpolation;
	bool calculateHF;
	vector<ImportedBody*> setBodies;
};
// Implemented by Luis Lee <End>
}