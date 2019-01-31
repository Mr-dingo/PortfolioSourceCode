#pragma once

#include <Util/Utils/XMLUtils.h>
#include <Body/Measurement/BodyMeasurement.h>
#include <Body/Measurement/BodyMeasureStruct.h>
#include <Editor/ModelBody/ModelBody.h>

namespace dcsbodyspace {

	enum GEN_TYPE { GEN_LENGTH = 0, GEN_ROTATION_X, GEN_ROTATION_Y, GEN_ROTATION_Z , GEN_GRITH, GEN_GRITH_X, GEN_GRITH_Y};

	class DCBODYDLL GenerationRuleAtom {
	public:
		GenerationRuleAtom() {}
		GenerationRuleAtom(xmlNodePtr node) {}
		~GenerationRuleAtom() {}

		// Implemented by Luis Lee <Begin>
		virtual void ImportXmlNode(xmlNodePtr) = 0;
		virtual void update() = 0;
		virtual void fit(){};
		// Implemented by Luis Lee <End>

		std::string			name;
		std::string			tag;
		int					idx;
		double				deform_value;
		double				old_deform_value;

		double				deform_percentage;
		double				original_measure;

		BasicBody*			body;
		BodyMeasurement*	bm; //just pointing..
		BodyMeasureStruct*	bms; //just pointing..
		vector<ArrayN<real>>	manCustomWeightBust;
		vector<ArrayN<real>>	manCustomWeightWaist;
		vector<ArrayN<real>>	man_custom_weight_knee;
		real fit_deform_value;

		static std::string XmlElementName() { return "Gen_Rule"; }
	};

	class DCBODYDLL LengthRuleAtom: public GenerationRuleAtom {
	public:
		LengthRuleAtom();
		LengthRuleAtom(xmlNodePtr node);
		~LengthRuleAtom();

		void ImportXmlNode(xmlNodePtr);
		void update();

		virtual void fit();
		int											deform_type;
		std::vector<string>							gen_bone_list;
		std::vector<Matrix4r>						gen_bone_list_RM;
		std::vector<real>							original_len;
		std::vector<double>							gen_bone_list_value;
		std::vector<Vector3r>						dir_local;
		std::string									parent;
		int											dir_parent;

		static std::string XmlElementName() { return "Length_Rule"; }
	};

	class DCBODYDLL HeightRuleAtom: public LengthRuleAtom {
	public:
		virtual void fit();
		HeightRuleAtom();
		HeightRuleAtom(xmlNodePtr node);
		~HeightRuleAtom();
		void update();
		static std::string XmlElementName() { return "Height_Rule"; }
	};

	class DCBODYDLL BreadthRuleAtom: public LengthRuleAtom {
	public:
		BreadthRuleAtom();
		BreadthRuleAtom(xmlNodePtr node);
		~BreadthRuleAtom();
		virtual void fit();
		void update();
		static std::string XmlElementName() { return "Breadth_Rule"; }
	};

	class DCBODYDLL DepthRuleAtom: public LengthRuleAtom {
	public:
		DepthRuleAtom();
		DepthRuleAtom(xmlNodePtr node);
		~DepthRuleAtom();
		void update();
		static std::string XmlElementName() { return "Depth_Rule"; }
	};

	class DCBODYDLL CircumRuleAtom: public GenerationRuleAtom {
	public:
		CircumRuleAtom();
		CircumRuleAtom(xmlNodePtr node);
		~CircumRuleAtom();

		void ImportXmlNode(xmlNodePtr);
		virtual void fit();
		void update();
		void set_vert(ArrayN<Vector3r,0>* v,ArrayN<Vector3r,0>* v_h,ArrayN<Vector3r,0>* r_h,ArrayN<Vector3r,0>* l_h,ArrayN<Vector3r,0>* r_f,ArrayN<Vector3r,0>* l_f)
			{original_vert=v;original_vert_head=v_h;original_vert_rhand=r_h;original_vert_lhand=l_h;original_vert_rfoot=r_f;original_vert_lfoot=l_f;}

		int						deform_type;
		int						weight_type;   
		std::vector<string>		gen_bone_list;
		std::vector<double>		gen_bone_list_value;
		ArrayN<Vector3r,0>*		original_vert;
		ArrayN<Vector3r,0>*		original_vert_head;
		ArrayN<Vector3r,0>*		original_vert_rhand;
		ArrayN<Vector3r,0>*		original_vert_lhand;
		ArrayN<Vector3r,0>*		original_vert_rfoot;
		ArrayN<Vector3r,0>*		original_vert_lfoot;
		vector<ArrayN<real>>	custom_weight;
		static std::string XmlElementName() { return "Circum_Rule"; }
	};
};