#include <Editor/SetBody/SetBody.h>
#include <Editor/GenerationRule/GenerationRule.h>
#include <math.h>

using namespace std;
using namespace dcsbodyspace;

SetBody::SetBody()
: BasicBody()
{
	setBodies.clear();
}
SetBody::~SetBody()
{
	for(int i=0; i < setBodies.size(); i++)
		if(setBodies[i]) delete setBodies[i];
}
bool SetBody::load(const std::wstring& wfilename)
{
	string filename = StringLibrary::wstring2string(wfilename);
	//bool success=BasicBody::load(filename);
	//cout<<"this is set Body!"<<endl;
	//return success;

	cout<<"this is set Body!"<<endl;
	string filenameOnly = DataPath::getFilenameOnly(filename);
	StringLibrary::replace(filenameOnly, "_body.iBody", "");
	StringLibrary::replace(filenameOnly, "_body.ibody", "");
	
	StringLibrary::replace(filenameOnly, "DC", "");
	if (filenameOnly[0]=='M') {
		bodytype = 1;
		StringLibrary::replace(filenameOnly, "MS_", "");
	}
	else if(filenameOnly[0]=='F') {
		bodytype = 0;
		StringLibrary::replace(filenameOnly, "FS_", "");
	}
	else if(filenameOnly[0]=='C') {
		bodytype = 2;
		StringLibrary::replace(filenameOnly, "CS_", "");
	}
	else
		bodytype = -1;
	//index = atoi(filenameOnly.c_str());

	if (body)
		delete body;
	body = new ImportedBody();
	body->load(wfilename);
	string filenameTemp = filename;
	string filenameOnlyTemp = DataPath::getFilenameOnly(filename);
	StringLibrary::replace(filenameOnlyTemp, "_body.iBody", "");
	StringLibrary::replace(filenameOnlyTemp, "_body.ibody", "");
	string firstBodyFilename = filenameOnlyTemp;
	bodyname = filenameOnlyTemp;
	StringLibrary::replace(filenameOnlyTemp, "_SS", "_");
	StringLibrary::replace(filenameOnlyTemp, "_SM", "_");
	StringLibrary::replace(filenameOnlyTemp, "_SF", "_");
	StringLibrary::replace(filenameOnlyTemp, "_MS", "_");
	StringLibrary::replace(filenameOnlyTemp, "_MM", "_");
	StringLibrary::replace(filenameOnlyTemp, "_MF", "_");
	StringLibrary::replace(filenameOnlyTemp, "_TS", "_");
	StringLibrary::replace(filenameOnlyTemp, "_TM", "_");
	StringLibrary::replace(filenameOnlyTemp, "_TF", "_");

	StringLibrary::replace(bodyname, "_SS", "");
	StringLibrary::replace(bodyname, "_SM", "");
	StringLibrary::replace(bodyname, "_SF", "");
	StringLibrary::replace(bodyname, "_MS", "");
	StringLibrary::replace(bodyname, "_MM", "");
	StringLibrary::replace(bodyname, "_MF", "");
	StringLibrary::replace(bodyname, "_TS", "");
	StringLibrary::replace(bodyname, "_TM", "");
	StringLibrary::replace(bodyname, "_TF", "");

	ImportedBody* SS = new ImportedBody();
	ImportedBody* SM = new ImportedBody();
	ImportedBody* SF = new ImportedBody();
	ImportedBody* MS = new ImportedBody();
	ImportedBody* MM = new ImportedBody();
	ImportedBody* MF = new ImportedBody();
	ImportedBody* TS = new ImportedBody();
	ImportedBody* TM = new ImportedBody();
	ImportedBody* TF = new ImportedBody();

	if(firstBodyFilename != filenameOnlyTemp+"SS")
		StringLibrary::replace(filenameTemp, firstBodyFilename, filenameOnlyTemp+"SS");
	SS->load(StringLibrary::string2wstring(filenameTemp));
	setBodies.push_back(SS);
	cout<<"push SS"<<endl;
	StringLibrary::replace(filenameTemp, filenameOnlyTemp+"SS", filenameOnlyTemp+"SM");
	SM->load(StringLibrary::string2wstring(filenameTemp));
	setBodies.push_back(SM);
	StringLibrary::replace(filenameTemp, filenameOnlyTemp+"SM", filenameOnlyTemp+"SF");
	SF->load(StringLibrary::string2wstring(filenameTemp));
	setBodies.push_back(SF);
	StringLibrary::replace(filenameTemp, filenameOnlyTemp+"SF", filenameOnlyTemp+"MS");
	MS->load(StringLibrary::string2wstring(filenameTemp));
	setBodies.push_back(MS);
	StringLibrary::replace(filenameTemp, filenameOnlyTemp+"MS", filenameOnlyTemp+"MM");
	MM->load(StringLibrary::string2wstring(filenameTemp));
	setBodies.push_back(MM);
	StringLibrary::replace(filenameTemp, filenameOnlyTemp+"MM", filenameOnlyTemp+"MF");
	MF->load(StringLibrary::string2wstring(filenameTemp));
	setBodies.push_back(MF);
	StringLibrary::replace(filenameTemp, filenameOnlyTemp+"MF", filenameOnlyTemp+"TS");
	TS->load(StringLibrary::string2wstring(filenameTemp));
	setBodies.push_back(TS);
	StringLibrary::replace(filenameTemp, filenameOnlyTemp+"TS", filenameOnlyTemp+"TM");
	TM->load(StringLibrary::string2wstring(filenameTemp));
	setBodies.push_back(TM);
	StringLibrary::replace(filenameTemp, filenameOnlyTemp+"TM", filenameOnlyTemp+"TF");
	TF->load(StringLibrary::string2wstring(filenameTemp));
	setBodies.push_back(TF);

	for(int i=0; i<setBodies.size(); i++) {
		setBodies[i]->load_motion_in_motion_file_list(0);
		setBodies[i]->update();
		setBodies[i]->set_frame(i);
	}


	interpolation = false;


	calculateHF = false;

	return true;
}
void SetBody::update()
{
	if(body)
		body->update();

	if(interpolation)
	{
		
		ImportedBody *model_a = setBodies[1];
		ImportedBody *model_b = setBodies[8];
		ArrayN<Vector3r,0> modelA=model_a->riggingObject->get_posVert();
		ArrayN<Vector3r,0> modelA_h=model_a->head->riggingObject->get_posVert();
		ArrayN<Vector3r,0> modelB=model_b->riggingObject->get_posVert();
		ArrayN<Vector3r,0> modelB_h=model_b->head->riggingObject->get_posVert();
		//body->riggingObject->getSkeleton()->m_pBoneList[0].length=0;// length ���⼭ �ٲ�.

		ArrayN<Vector3r,0> modelA_rHand = model_a->Body_RHand->riggingObject->get_posVert();
		ArrayN<Vector3r,0> modelA_lHand =model_a->Body_LHand->riggingObject->get_posVert();
		ArrayN<Vector3r,0> modelA_rFoot =model_a->Body_RFoot->riggingObject->get_posVert();
		ArrayN<Vector3r,0> modelA_lFoot =model_a->Body_LFoot->riggingObject->get_posVert();

		ArrayN<Vector3r,0> modelB_rHand = model_b->Body_RHand->riggingObject->get_posVert();
		ArrayN<Vector3r,0> modelB_lHand =model_b->Body_LHand->riggingObject->get_posVert();
		ArrayN<Vector3r,0> modelB_rFoot =model_b->Body_RFoot->riggingObject->get_posVert();
		ArrayN<Vector3r,0> modelB_lFoot =model_b->Body_LFoot->riggingObject->get_posVert();



		cout<<model_b->body_accessories->filename<<endl;
		cout<<model_b->body_other_nodes->filename<<endl;




		ArrayN<Bone,0> modelA_bone = model_a->riggingObject->getSkeleton()->m_pBoneList;
		ArrayN<Bone,0> modelB_bone = model_b->riggingObject->getSkeleton()->m_pBoneList;

		cout<<body->riggingObject->get_posVert_base().Size()<<endl;
		cout<<model_a->riggingObject->get_posVert().Size()<<endl;
		cout<<model_b->riggingObject->get_posVert().Size()<<endl;
		//body->Body_LFoot->riggingObject->get_posVert_base();

		real a =2;
		cout<<"size"<<endl;
		ArrayN<int,0> index;
		index.Resize(body->riggingObject->get_posVert_base().Size());

		double tmp = 0;//average value of distance between model1 and model2
		for(int i = 0; i <body->riggingObject->get_posVert_base().Size(); i++)
		{
			tmp += (modelA[i] - modelB[i]).length();
			index[i] = (modelA[i] - modelB[i]).length();
		}
		tmp = tmp/body->riggingObject->get_posVert_base().Size();

		for( int i =0 ; i < body->riggingObject->get_posVert_base().Size() ; i++)
		{
			if(index[i] < tmp*2)
			{
				modelA[i] = (modelA[i]+modelB[i])/a;
			}
			else
			{
				cout<<"too far"<<endl;
			}
		}
		for( int i =0 ; i < body->head->riggingObject->get_posVert_base().Size() ; i++)
		{
			modelA_h[i] = (modelA_h[i]+modelB_h[i])/a;
		}
		body->riggingObject->setposvert(modelA);	
		body->head->riggingObject->setposvert(modelA_h);

		//accessaries
		for(int i = 0 ; i<body->Body_RHand->riggingObject->get_posVert().Size(); i++)
		{
			modelA_rHand[i] =( modelA_rHand[i] + modelB_rHand[i] )/a;
		}
		for(int i = 0 ; i<body->Body_LHand->riggingObject->get_posVert().Size(); i++)
		{
			modelA_lHand[i] =( modelA_lHand[i] + modelB_lHand[i] )/a;
		}
		for(int i = 0 ; i<body->Body_RFoot->riggingObject->get_posVert().Size(); i++)
		{
			modelA_rFoot[i] =( modelA_rFoot[i] + modelB_rFoot[i] )/a;
		}
		for(int i = 0 ; i<body->Body_LFoot->riggingObject->get_posVert().Size(); i++)
		{
			modelA_lFoot[i] =( modelA_lFoot[i] + modelB_lFoot[i] )/a;
		}
		cout<<"hair index = "<<body->hair_idx<<endl;
		cout<<(body->body_accessories->get_body_accessory_list())[0]->boneName<<endl;
		//skeleton
		for(int i = 0 ; i<model_a->riggingObject->getSkeleton()->m_pBoneList.Size(); i++)
		{
			body->riggingObject->getSkeleton()->m_pBoneList[i].length =( modelA_bone[i].length + modelB_bone[i].length )/a;
			body->riggingObject->getSkeleton()->m_pBoneList[i].length_ori =( modelA_bone[i].length_ori + modelB_bone[i].length_ori )/a;
		}
		body->Body_LFoot->riggingObject->setposvert(modelA_lFoot);
		body->Body_RFoot->riggingObject->setposvert(modelA_rFoot);
		body->Body_LHand->riggingObject->setposvert(modelA_lHand);
		body->Body_RHand->riggingObject->setposvert(modelA_rHand);
		interpolation = false;
	}

}
