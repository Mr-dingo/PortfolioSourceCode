#pragma once
#include <Util/Utils/XMLUtils.h>
#include <Editor/GenerationRule/GenerationRuleAtom.h>

namespace dcsbodyspace {

class DCBODYDLL GenerationRule {
public:
	GenerationRule();
	~GenerationRule();
	void Import(const std::string&);
	virtual void ImportXmlNode(xmlNodePtr) = 0;
	GenerationRuleAtom* FindGenRule(std::string&);
	void update();
	void setting_bm(BodyMeasureStruct** bm0);
	std::string Filename;
	int numgen;
	std::vector<GenerationRuleAtom*> Genlist;
	static std::string XmlElementName() { return "Gen_Rule"; }
};

class DCBODYDLL Length_GenRule : public  GenerationRule {
public:
	Length_GenRule();
	~Length_GenRule();

	void ImportXmlNode(xmlNodePtr);

	std::vector<double> bone_deform_value;
	std::vector<Matrix4r> bone_RM;
	static std::string XmlElementName() { return "Gen_Rule"; }
};

class DCBODYDLL Breadth_GenRule : public  Length_GenRule {
public:
	Breadth_GenRule();
	~Breadth_GenRule();
	void ImportXmlNode(xmlNodePtr);
	static std::string XmlElementName() { return "Gen_Rule"; }
};

class DCBODYDLL Height_GenRule : public  Length_GenRule {
public:
	Height_GenRule();
	~Height_GenRule();
	void ImportXmlNode(xmlNodePtr);
	static std::string XmlElementName() { return "Gen_Rule"; }
};

class DCBODYDLL Depth_GenRule : public  Length_GenRule {
public:
	Depth_GenRule();
	~Depth_GenRule();
	void ImportXmlNode(xmlNodePtr);
	static std::string XmlElementName() { return "Gen_Rule"; }
};

class DCBODYDLL Circum_GenRule : public  GenerationRule {
public:
	Circum_GenRule();
	~Circum_GenRule();

	void ImportXmlNode(xmlNodePtr);
	std::vector<std::vector<int>> gen_bone_enable_weight;
	ArrayN<Vector3r, 0> original_vert;
	ArrayN<Vector3r, 0> original_vert_head;
	ArrayN<Vector3r, 0> original_vert_rhand;
	ArrayN<Vector3r, 0> original_vert_lhand;
	ArrayN<Vector3r, 0> original_vert_rfoot;
	ArrayN<Vector3r, 0> original_vert_lfoot;

	static std::string XmlElementName() { return "Gen_Rule"; }
};

class DCBODYDLL BodyGenRules {
public:
	BodyGenRules();
	~BodyGenRules();

	virtual void initialize() = 0;
	virtual void load(bool isMale) = 0;
	virtual void update() = 0;
	virtual void setBms(BodyMeasureStruct** bm0) { bms=bm0; }
	virtual void setBody(BasicBody* b) { body=b; }

	BodyMeasureStruct** bms;
	BasicBody* body;
	ArrayN<real,0> lengthAdjustmentSize;
	ArrayN<real,0> heightAdjustmentSize;
	ArrayN<real,0> breadthAdjustmentSize;
	ArrayN<real,0> depthAdjustmentSize;
	ArrayN<real,0> circumAdjustmentSize;
	float mFatness;
	float mHeight;
};

class DCBODYDLL ModelBodyGenRules : public BodyGenRules{
public:
	ModelBodyGenRules();
	~ModelBodyGenRules();
	void					initialize();
	void					load(bool isMale);
	void					update();
	void					setBms(BodyMeasureStruct** bm0) { bms=bm0; }


	// Implemented by Luis Lee <Begin>
	void					setBody(BasicBody* b) { body=b; }
	void					ImportXmlNode(xmlNodePtr xml_node) ;
	int						sizeType2index(string sizeType);
	void					Read_XML_FILE(wstring filename);
	static					std::string XmlElementName() { return "MEASUREDEF"; }
	void					Read_CSV_FILE(wstring filename);
	double					manCircumWeight(double axilla, double breast, double x );
	double					smoothingAxisX(double a, double b, double x);
	void					bustWeightGen();
	void					waistWeightGen();
	void					kneeWeightGen();
	void					preFit();
	void					exactFit();
	void					exactFitSecond();
	void					inputAdjustmentSize(wstring dataName);
	void					calculateHF();
	void					analyzeSize(wstring outputFile);
	void					writeWholeSize(wstring outputFile);
	// Implemented by Luis Lee <End>
private:
	Length_GenRule*			lengthGenRule;
	Circum_GenRule*			circumGenRule;
	Height_GenRule*			heightGenRule;
	Breadth_GenRule*		breadthGenRule;
	Depth_GenRule*			depthGenRule;
	bool					fitted;
	int						counter;
};
// Implemented by Luis Lee <Begin>
class DCBODYDLL SetBodyGenRules : public BodyGenRules{
public:
	SetBodyGenRules();
	~SetBodyGenRules();
	void initialize();
	void load(bool isMale);
	void update();
	void setBms(BodyMeasureStruct** bm0) { bms=bm0; }
	void setBody(BasicBody* b) { body=b; }
	void interpolationFatness(RiggingObject_Base* rigBase,RiggingObject_Base* rig_0_0,RiggingObject_Base* rig_0_1,RiggingObject_Base* rig_1_0,RiggingObject_Base* rig_1_1);
	void interpolationHeight(RiggingObject_Base* rigBase,RiggingObject_Base* rig_0_0,RiggingObject_Base* rig_0_1,RiggingObject_Base* rig_1_0,RiggingObject_Base* rig_1_1);
	void interpolationFatnessForHands(RiggingObject_Base* rigBase,RiggingObject_Base* rig_0_0,RiggingObject_Base* rig_0_1,RiggingObject_Base* rig_1_0,RiggingObject_Base* rig_1_1);

};
// Implemented by Luis Lee <End>
};