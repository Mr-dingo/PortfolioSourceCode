#include <csvLoader/csvLoader.h>
#include <Editor/GenerationRule/GenerationRule.h>
#include <cmath>

using namespace std;
using namespace dcsbodyspace;

void ModelBodyGenRules::calculateHF(){
	real lengthArray[5][3] = {//S-M-T
		{148.2,  160.5 ,172.6 },
		{ 88.4,97.5 ,107.6},
		{31.4, 37.8 ,43.0 },
		{ 90.6,100.3 , 110.5},
		{49.2, 54.4,60.3 }

	};
	real fatnessArray[15][3] = {//S-M-F
		{23.8 ,27.1 ,32.0},
		{23.2 ,26.2 ,31.4},
		{ 22.7 ,26.4, 32.7},
		{ 28.8 ,32.2, 36.1},
		{14.5, 17.6, 22.7},
		{16.9, 20.4, 26.4  },
		{13.5, 16.9, 23.1},
		{18.0, 21.0, 27.1  },
		{73.4, 82.0, 99.6 },
		{73.0, 82.1, 101.5},
		{63.8, 71.2, 87.4},
		{58.4, 68.9, 89.2},
		{82.0, 90.5, 105.1 },
		{46.3, 54.3, 64.6 },
		{20.9, 24.5, 32.3}
	};
	if(body->getbodyType() == 4 ||body->getbodyType() == 1 ){
		lengthArray[0][0] = 165.6;
		lengthArray[0][1] =  173.8;
		lengthArray[0][2] = 182.2;

		fatnessArray[9][0] = 86.5;
		fatnessArray[9][1] =  94.4;
		fatnessArray[9][2] = 103.4;

		fatnessArray[12][0] = 89;
		fatnessArray[12][1] = 94.3;
		fatnessArray[12][2] = 100;
	}

	real lengthWeight[5] = {5,0.5,0.1,0.5,0.3};
	real fatnessWeight[15] = {0.1,0.1,0.1,0.1,0.1,1,1,1,1,5,5,5,5,0.6,0.5};
	
	real userLengthArray[5]={heightAdjustmentSize[0],NULL,NULL,NULL,NULL}; //stature
	real userFatnessArray[15]={
		NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
		circumAdjustmentSize[1],NULL,NULL,
		circumAdjustmentSize[3],NULL,NULL}; //bust, hip circum
	real lengthResult[5];
	real fatnessResult[15];
	for(int i=0; i<5;i++){//Height
		if(userLengthArray[i]==NULL){
			lengthResult[i]=NULL;
		}
		else{
			if(userLengthArray[i]-lengthArray[i][1]>0){//M-T
				lengthResult[i] = 1+(userLengthArray[i]-lengthArray[i][1])/(lengthArray[i][2]-lengthArray[i][1]);
			}
			else{//S-M
				lengthResult[i]=1+(userLengthArray[i]-lengthArray[i][1])/(lengthArray[i][1]-lengthArray[i][0]);
			}			
		}
	}
	for(int i=0; i<15;i++){//Fatness
		if(userFatnessArray[i]==NULL){
			fatnessResult[i]=NULL;
		}
		else{
			if(userFatnessArray[i]-fatnessArray[i][1]>0){//M-F
				fatnessResult[i] = 1+(userFatnessArray[i]-fatnessArray[i][1])/(fatnessArray[i][2]-fatnessArray[i][1]);
			}
			else{//S-M
				fatnessResult[i]=1+(userFatnessArray[i]-fatnessArray[i][1])/(fatnessArray[i][1]-fatnessArray[i][0]);
			}			
		}
	}
	real tmpH=0;
	real tmpF=0;
	real tmpHw=0;
	real tmpFw=0;
	for(int i=0;i<5;i++){
		if(lengthResult[i] !=NULL){
			tmpH += lengthResult[i]*lengthWeight[i];
			tmpHw += lengthWeight[i];
		}
	}
	for(int i=0;i<15;i++){
		if(fatnessResult[i] !=NULL){
			tmpF += fatnessResult[i]*fatnessWeight[i];
			tmpFw += fatnessWeight[i];
		}
	}
	mHeight = tmpH/tmpHw;
	mFatness =tmpF/tmpFw;
	mHeight = dcsbodyspace::clamp(mHeight,0.0f,2.0f);
	mFatness = dcsbodyspace::clamp(mFatness,0.0f,2.0f);
}

void ModelBodyGenRules::inputAdjustmentSize(wstring dataName){
	const int MAX_CHARS_PER_LINE = 512;
	const int MAX_TOKENS_PER_LINE = 20;
	const char* const DELIMITER = " ";

	circumAdjustmentSize.Resize(circumGenRule->Genlist.size());
	heightAdjustmentSize.Resize(heightGenRule->Genlist.size());
	depthAdjustmentSize.Resize(depthGenRule->Genlist.size());
	breadthAdjustmentSize.Resize(breadthGenRule->Genlist.size());
	lengthAdjustmentSize.Resize(lengthGenRule->Genlist.size());

	circumAdjustmentSize.MemsetZero();
	heightAdjustmentSize.MemsetZero();
	depthAdjustmentSize.MemsetZero();
	breadthAdjustmentSize.MemsetZero();
	lengthAdjustmentSize.MemsetZero();

	std::size_t found = dataName.find(L".xml");
	if(found!=std::string::npos){
		Read_XML_FILE(dataName);
		return;
	}
	std::size_t foundCsv = dataName.find(L".csv");
	if(foundCsv!=std::string::npos){
		Read_CSV_FILE(dataName);
		return;
	}

	int size = 11;
	real dataMeasure[11];
	ifstream fin;
	wstring wDataName;
	wDataName = L"";
	wDataName.assign(dataName.begin(),dataName.end());
	fin.open(dataName); // open a file
	if (!fin.good()) {
		std::wcout<<"cannot found file :"<<dataName<<endl;
		
	}
	else{
		int index=0;
		// read each line of the file
		while (!fin.eof())
		{
		// read an entire line into memory
		char buf[MAX_CHARS_PER_LINE];
		fin.getline(buf, MAX_CHARS_PER_LINE);
	
		// parse the line into blank-delimited tokens
		int n = 0; // a for-loop index
	
		// array to store memory addresses of the tokens in buf
		const char* token[MAX_TOKENS_PER_LINE]={ }; // initialize to 0
		
		// parse the line
		token[0] = strtok(buf, DELIMITER); // first token
		if (token[0]) // zero if line is blank
		{
			for (n = 1; n < MAX_TOKENS_PER_LINE; n++)
			{
				token[n] = strtok(0, DELIMITER); // subsequent tokens
				if (!token[n]) break; // no more tokens
			}
		}
		
		if(token[0]){
			if(token[2])
			{

						dataMeasure[index] = atof(token[2]);	
						if(std::string(token[0]) == "Circum") 
						{
							circumAdjustmentSize[index] = dataMeasure[index];
						}

						else if(std::string(token[0]) == "Length")
						{
							lengthAdjustmentSize[6] = (dataMeasure[index]); 
						}

						else if(std::string(token[0]) == "Height")
						{
							if(std::string(token[1]) == "Stature")
							{
								heightAdjustmentSize[0] =  (dataMeasure[index]); 
							}
							else if(std::string(token[1])=="Upper_Body_Height")
							{
								heightAdjustmentSize[1] =  (dataMeasure[index]); 
							}
						}

						else if(std::string(token[0]) == "Breadth")
						{
							breadthAdjustmentSize[0] = (dataMeasure[index]);
						}
						else if(std::string(token[0]) == "Body_Type")
						{
							lengthAdjustmentSize[0] = (dataMeasure[index]);
						}
						else
						{

						}
						index++;
			}
			else{
				dataMeasure[index] = NULL;	
						if(std::string(token[0]) == "Circum")
						{
							circumAdjustmentSize[index] = dataMeasure[index];
						}

						else if(std::string(token[0]) == "Length")
						{
							lengthAdjustmentSize[6] = (dataMeasure[index]); 
						}

						else if(std::string(token[0]) == "Height")
						{
							if(std::string(token[1]) == "Stature")
							{
								heightAdjustmentSize[0] =  (dataMeasure[index]); 
							}
							else if(std::string(token[1])=="Upper_Body_Height")
							{
								heightAdjustmentSize[1] =  (dataMeasure[index]); 
							}
						}

						else if(std::string(token[0]) == "Breadth")
						{
							breadthAdjustmentSize[0] = (dataMeasure[index]);
						}
						else if(std::string(token[0]) == "Body_Type")
						{
							lengthAdjustmentSize[0] = (dataMeasure[index]); 
						}
						else
						{

						}
						index++;
			}

		}
		if(index == size)
			break;
	  }
  }
  fin.close();
	
}

void ModelBodyGenRules::ImportXmlNode(xmlNodePtr xmlNode){	
	string TEXT = "text";
	xmlNodePtr next = xmlFindNode(xmlNode, BAD_CAST "MESURE");
	for (xmlNodePtr curNode = xmlNode->children; curNode; curNode = curNode->next) {
		if (xmlStrcmp(curNode->name, BAD_CAST TEXT.c_str()) != 0) {
			char* typeIndex = ((char*)xmlFindNode(curNode, BAD_CAST "codemesure")->children->content);
			string typeName = string((char*)xmlFindNode(curNode, BAD_CAST "nommesure")->children->content);
			char* value_ = ((char*)xmlFindNode(curNode, BAD_CAST "valeurmesure")->children->content);
			int typeNum = atoi(typeIndex);
			float sizeVal = atof(value_);
			switch (typeNum)
			{
			case 1: //heigt
				heightAdjustmentSize[sizeType2index("Stature")] = sizeVal;
				break;
			case 42:// arm length
				lengthAdjustmentSize[sizeType2index("Arm_Length")] = sizeVal;
				break;
			case 3://Neck_circum
				circumAdjustmentSize[sizeType2index("Neck_circum")]=sizeVal;
				break;
			case 22: // Thigh_circum
				circumAdjustmentSize[sizeType2index("Thigh_circum")]=sizeVal;
				break;
			case 4: //chest girth
				circumAdjustmentSize[sizeType2index("Bust_circum")]=sizeVal;
				break;
			case 13: //wiast girt
				circumAdjustmentSize[sizeType2index("Wasit_circum")]=sizeVal;
				break;
			case 16: // hip girth
				circumAdjustmentSize[sizeType2index("Hip_circum")]=sizeVal;
				break;
			case 24: //knee girth
				circumAdjustmentSize[sizeType2index("Knee_circum")]=sizeVal;
				break;
			case 4157: //Bishoulder breadth
				breadthAdjustmentSize[sizeType2index("Bishoulder_breadth")] = sizeVal;
				break;
			case 4159: // upper mHeight 
				heightAdjustmentSize[sizeType2index("Upper_Body_Height")] = sizeVal;
				break;
			default:
				break;
			}
		}
	}
	return;
}

int ModelBodyGenRules::sizeType2index(string name){// input : size name  output : size index in genRule
	if(name == "Neck_circum")
		return 0;
	if(name == "Bust_circum")
		return 1;
	if(name == "Wasit_circum")
		return 2;
	if(name == "Hip_circum")
		return 3;
	if(name == "Thigh_circum")
		return 4;
	if(name == "Knee_circum")
		return 5;
	if(name == "Arm_Length")
		return 6;
	if(name == "Stature")
		return 0;
	if(name == "Upper_Body_Height")
		return 1;
	if(name == "Bishoulder_breadth")
		return 0;
	if(name == "Stooped_Percentage") //length
		return 0;
	else
		return -1;

}
void ModelBodyGenRules::Read_XML_FILE(wstring filename){
	string s = "";
	s.assign(filename.begin(),filename.end());
	ImportXmlFile(this,s.c_str());
}

void ModelBodyGenRules::Read_CSV_FILE(wstring filename){
	
	ifstream fin;
	wstring wDataName;
	wDataName = L"";
	wDataName.assign(filename.begin(),filename.end());
	vector<string> token;
	string line;
	fin.open(filename); // open a file
	if (!fin.good()) {
		std::wcout<<"cannot found file :"<<filename<<endl;
		
	}
	else{
		
		int index=0;
		// read each line of the file
		while (!fin.eof())
		{
			string buf;
			getline(fin,buf);
			csvline_populate(token,buf,',');
			if (atof(token[1].c_str())) 
			{
				if(stricmp(token[0].c_str(),"Mid Neck Girth")==0)				{circumAdjustmentSize[sizeType2index("Neck_circum")] = atof(token[1].c_str());			}
				if(stricmp(token[0].c_str(),"Chest girth")==0)					{circumAdjustmentSize[sizeType2index("Bust_circum")] = atof(token[1].c_str());			}
				if(stricmp(token[0].c_str(),"Waist girth")==0)					{circumAdjustmentSize[sizeType2index("Wasit_circum")] = atof(token[1].c_str());			}
				if(stricmp(token[0].c_str(),"Low Hip girth")==0)				{circumAdjustmentSize[sizeType2index("Hip_circum")] = atof(token[1].c_str());				}
				if(stricmp(token[0].c_str(),"Left Knee girth")==0)				{circumAdjustmentSize[sizeType2index("Knee_circum")] = atof(token[1].c_str());			}
				if(stricmp(token[0].c_str(),"Left Thigh girth")==0)				{circumAdjustmentSize[sizeType2index("Thigh_circum")] = atof(token[1].c_str());			}
				if(stricmp(token[0].c_str(),"Center Back Neck to Waist")==0)	{heightAdjustmentSize[sizeType2index("Upper_Body_Height")] = atof(token[1].c_str());		}
				if(stricmp(token[0].c_str(),"Height")==0)						{heightAdjustmentSize[sizeType2index("Stature")] = atof(token[1].c_str());				}
				if(stricmp(token[0].c_str(),"SHOULDER HOW???")==0)				{breadthAdjustmentSize[sizeType2index("Bishoulder_breadth")] = atof(token[1].c_str());	}
				if(stricmp(token[0].c_str(),"Right upper arm length")==0)		{lengthAdjustmentSize[sizeType2index("Arm_Length")] = atof(token[1].c_str());				}					
			}
		}
	}
	fin.close();

}
void ModelBodyGenRules::preFit(){
	if(lengthAdjustmentSize[0] == NULL)
		lengthAdjustmentSize[0] = 0;
	real per;
	if(lengthAdjustmentSize[0] > 0)
		per = lengthAdjustmentSize[0]*0.3;
	else
		per = lengthAdjustmentSize[0]*0.3;
	if(per >30)
		per = 30.0f;
	else if(per < -30)
		per = -30.0f;
	lengthGenRule->Genlist[2]->fit_deform_value = per; 
	lengthGenRule->Genlist[2]->fit();
	if(circumAdjustmentSize[0]>1)	{circumAdjustmentSize[0] = clamp(float(circumAdjustmentSize[0]),25.0f,37.0f); }
	if(circumAdjustmentSize[1]>1)	{circumAdjustmentSize[1] = clamp(float(circumAdjustmentSize[1]),70.0f,120.0f);}
	if(heightAdjustmentSize[1]>1)	{heightAdjustmentSize[1] = clamp(float(heightAdjustmentSize[1]),30.0f,48.0f); }
}

void ModelBodyGenRules::exactFit(){
	real boundary;
	fitted = false;
	boundary = 0.01f;
	real per = 0; //circum boundary condition
	real perCompare =0;
	//circum part
	for(int i = 0 ; i< 6; i++)
	{
		if(circumAdjustmentSize[i] != NULL && circumAdjustmentSize[i] !=0)
		{
			if(i == 3)
				continue;
			else if(i==0)
			{
				per = ((circumAdjustmentSize[0] -circumGenRule->Genlist[circumGenRule->Genlist.size()-1]->bm->measure)/circumAdjustmentSize[0])*100.0f;//genlist 0 index �� neck_circum
				
				perCompare = max(per,perCompare);
				if(per >= boundary || per <=-1.f*boundary)
				{
						per = clamp(per,real(-5),real(5));
						circumGenRule->Genlist[0]->fit_deform_value = per;
						circumGenRule->Genlist[0]->fit();
						counter++;
				}
			}
			else
			{
				per = ((circumAdjustmentSize[i] -circumGenRule->Genlist[i]->bm->measure)/circumAdjustmentSize[i])*100.0f;
				perCompare = max(per,perCompare);
				if(per >= boundary || per <=-1.f*boundary)
				{
						circumGenRule->Genlist[i]->fit_deform_value = per;
						circumGenRule->Genlist[i]->fit();
						counter++;
				}
			}
		}
	}

	//mHeight part
	real perHeight =0;
	real perHeightComp =0;
	for(int i =0 ; i<2;i++)
	{
		if(heightAdjustmentSize[i] != NULL && heightAdjustmentSize[i] !=0)
		{
			perHeight = ((heightAdjustmentSize[i] -heightGenRule->Genlist[i]->bm->measure)/heightAdjustmentSize[i])*100.0f;
			perHeightComp = max(perHeight,perHeightComp);
			if(perHeight >= boundary*10||perHeight <=-1.f*boundary*10) 
			{
				heightGenRule->Genlist[i]->fit_deform_value = perHeight;
				heightGenRule->Genlist[i]->fit();		
				counter++;
			}
		}
	}




	//length part
	real perLength =0;
	real perLengthComp=0;
	for(int i = 0 ; i<8; i++){
		if(lengthAdjustmentSize[i] != NULL && lengthAdjustmentSize[i] !=0)
		{
			if(i==6)
			{
				perLength = ((lengthAdjustmentSize[i] -lengthGenRule->Genlist[i]->bm->measure)/lengthAdjustmentSize[i])*100.0f;
				perLengthComp = max(perLength,perLengthComp);
				if(perLength >= boundary || perLength <=-1.f*boundary)
				{
				lengthGenRule->Genlist[i]->fit_deform_value = perLength;
				lengthGenRule->Genlist[i]->fit();
				counter++;
				}
			}
		}
	}

	//breadth part
	real per4 =0;
	if(breadthAdjustmentSize[0] != NULL && breadthAdjustmentSize[0] !=0){
		per4 = ((breadthAdjustmentSize[0] -breadthGenRule->Genlist[0]->bm->measure)/breadthAdjustmentSize[0])*100.0f;
		if(per4 >= boundary||per4 <=-1.f*boundary){
				breadthGenRule->Genlist[0]->fit_deform_value = per4;
				breadthGenRule->Genlist[0]->fit();		
				counter++;
		}
	}
	if(perCompare <= boundary&&perCompare >=-1.f*boundary &&
		perHeightComp <= boundary*10 && perHeightComp >=-1.f*boundary*10 &&
		perLengthComp <= boundary&& perLengthComp >=-1.f*boundary &&
		per4 <= boundary&&per4 >=-1.f*boundary || counter > 200)
	{
		fitted = true;
	}

}

void ModelBodyGenRules::exactFitSecond(){
	real boundary;
	fitted = false;
	boundary = 0.01f;

	real perHeight =0;
	for(int i =0 ; i<1;i++)
	{
		if(heightAdjustmentSize[i] != NULL&& heightAdjustmentSize[i] !=0){
			if(i==0)
			{
				perHeight = ((heightAdjustmentSize[i] -heightGenRule->Genlist[i]->bm->measure)/heightAdjustmentSize[i])*100.0f;
				if(perHeight >= boundary||perHeight <=-1.f*boundary)
				{
					heightGenRule->Genlist[i]->fit_deform_value = perHeight;
					heightGenRule->Genlist[i]->fit();		
					counter++;
				}
			}
		}
		
	}
	real per = 0; //circum boundary condition
	//Hip _ circum part
	if(circumAdjustmentSize[3] != NULL)
	{
		per = ((circumAdjustmentSize[3] -circumGenRule->Genlist[3]->bm->measure)/circumAdjustmentSize[3])*100.0f;
		if(per >= boundary || per <=-1.f*boundary)
		{
			circumGenRule->Genlist[3]->fit_deform_value = per;
			circumGenRule->Genlist[3]->fit();
			counter++;
		}
	}
	if(per <= boundary&&per >=-1.f*boundary &&
		perHeight <= boundary&&perHeight >=-1.f*boundary ||counter > 500)
	{
		fitted = true;
	}
}


void ModelBodyGenRules::analyzeSize(wstring outputFile){
	std::size_t found = outputFile.find(L".xml"); 
	if(found!=std::string::npos)
	{
		outputFile[found] = '.';
		outputFile[found+1] = 't';
		outputFile[found+2] = 'x';
		outputFile[found+3] = 't';
	}
	ofstream of;
	of.open(outputFile);
	of<<"< mHeight , mFatness > = < "<<mHeight<<" , "<<mFatness<<" >"<<endl;
	of<<"Name---Output size---Input size"<<endl;
	of<< circumGenRule->Genlist[circumGenRule->Genlist.size()-1]->name <<" = " <<circumGenRule->Genlist[circumGenRule->Genlist.size()-1]->bm->measure <<" ->> "<<circumAdjustmentSize[0]<<endl;
	for(int i = 1 ; i < 6; i++)
		of<< circumGenRule->Genlist[i]->name <<" = " <<circumGenRule->Genlist[i]->bm->measure <<" ->> "<<circumAdjustmentSize[i]<<endl;
	of<< lengthGenRule->Genlist[6]->name <<" = " <<lengthGenRule->Genlist[6]->bm->measure <<" ->> "<<lengthAdjustmentSize[6]<<endl;
	of<< heightGenRule->Genlist[0]->name <<" = " <<heightGenRule->Genlist[0]->bm->measure <<" ->> "<<heightAdjustmentSize[0]<<endl;
	of<< heightGenRule->Genlist[1]->name <<" = " <<heightGenRule->Genlist[1]->bm->measure <<" ->> "<<heightAdjustmentSize[1]<<endl;
	of<< breadthGenRule->Genlist[0]->name <<" = " <<breadthGenRule->Genlist[0]->bm->measure <<" ->> "<<breadthAdjustmentSize[0]<<endl;
	of<< "Body_Type Stooped_Percentage = "<<int(lengthGenRule->Genlist[2]->deform_value*1000/3)<<endl;

	of.close();
}

void ModelBodyGenRules::writeWholeSize(wstring outputFile){
	ofstream of;
	of.open(outputFile);
	of<<"Circum "<< circumGenRule->Genlist[circumGenRule->Genlist.size()-1]->name <<" " <<circumGenRule->Genlist[circumGenRule->Genlist.size()-1]->bm->measure<<endl;
	for(int i = 1 ; i < 6; i++)
	{
		of<<"Circum ";
		of<< circumGenRule->Genlist[i]->name <<" " <<circumGenRule->Genlist[i]->bm->measure<<endl;
	}
	of<<endl;
	of<<"Length "<< lengthGenRule->Genlist[6]->name <<" " <<lengthGenRule->Genlist[6]->bm->measure<<endl;
	of<<endl;
	of<< "Height "<<heightGenRule->Genlist[0]->name <<" " <<heightGenRule->Genlist[0]->bm->measure<<endl;
	of<< "Height "<<heightGenRule->Genlist[1]->name <<" " <<heightGenRule->Genlist[1]->bm->measure<<endl;
	of<<endl;
	of<< "Breadth "<<breadthGenRule->Genlist[0]->name <<" " <<breadthGenRule->Genlist[0]->bm->measure<<endl;
	of<<endl;
	of<<"Body_Type Stooped_Percentage "<< int(lengthGenRule->Genlist[2]->deform_value*1000/3)<<endl;
	of<<"\n"<<endl;
	of<<"*****Whole SIZE*****"<<endl;
	of<<"< mHeight , mFatness > = < "<<mHeight<<" , "<<mFatness<<" >"<<endl;
	for(int i=0;i<circumGenRule->Genlist.size();i++)
		of<< circumGenRule->Genlist[i]->name <<" = " <<circumGenRule->Genlist[i]->bm->measure <<endl;
	of<<endl;
	for(int i=0;i<lengthGenRule->Genlist.size();i++)
		of<< lengthGenRule->Genlist[i]->name <<" = " <<lengthGenRule->Genlist[i]->bm->measure <<endl;
	of<<endl;
	for(int i=0;i<heightGenRule->Genlist.size();i++)
		of<< heightGenRule->Genlist[i]->name <<" = " <<heightGenRule->Genlist[i]->bm->measure <<endl;
	of<<endl;
	for(int i=0;i<breadthGenRule->Genlist.size();i++)
		of<< breadthGenRule->Genlist[i]->name <<" = " <<breadthGenRule->Genlist[i]->bm->measure <<endl;
	of.close();
}

double ModelBodyGenRules::manCircumWeight(double axilla, double breast, double x ){//bust wieght, waist weight => axilla landmark > breast landmark 
	double nearAxilla = std::pow(x-axilla,3);
	double nearBreast = std::pow(x-breast,3);
	double tmp = -1*nearAxilla*nearBreast;
	if(tmp >= 0)
		return sqrt(tmp);
	else
		return 0;
}
double ModelBodyGenRules::smoothingAxisX(double a, double b, double x){//only for bust landmark
	double tmp = -1* (x-a) * (x-b);
	if(tmp >= 0)
		return sqrt(tmp)/1000;
	else
		return 0;
}
void ModelBodyGenRules::bustWeightGen(){
	ArrayN<real> manCircumWeight;
	
	Vector3r RtAxillaPos = circumGenRule->Genlist[1]->bms->BLs->blList[(circumGenRule->Genlist[1]->bms->BLs->GetLandmarkIdx("Rt_Axilla"))].pos; //x axis
	Vector3r LtAxillaPos = circumGenRule->Genlist[1]->bms->BLs->blList[circumGenRule->Genlist[1]->bms->BLs->GetLandmarkIdx("Lt_Axilla")].pos; // x axis
	Vector3r RtAnteriorMidaxillaPos = circumGenRule->Genlist[1]->bms->BLs->blList[circumGenRule->Genlist[1]->bms->BLs->GetLandmarkIdx("Rt_Lateral_Shoulder")].pos; //y axis
	Vector3r RtLateralWaistNaturalIndentationPos =circumGenRule->Genlist[1]->bms->BLs->blList[circumGenRule->Genlist[1]->bms->BLs->GetLandmarkIdx("Rt_Lateral_Waist_natural_indentation")].pos; //y axis
	Vector3r LtLateralWaistNaturalIndentationPos = circumGenRule->Genlist[1]->bms->BLs->blList[circumGenRule->Genlist[1]->bms->BLs->GetLandmarkIdx("Lt_Lateral_Waist_natural_indentation")].pos; //z axis

	ArrayN<Vector3r,0> a=body->getBody()->riggingObject->get_posVert_base();
	manCircumWeight.Resize(a.Size());
	int tmp = 0;
	double max = 0;
	for(int i = 0; i < a.Size(); i++)
	{		
		if( RtAnteriorMidaxillaPos.y()>a[i].y() &&
			a[i].y()>RtLateralWaistNaturalIndentationPos.y() && 
			RtAxillaPos.x()<a[i].x() && 
			a[i].x()<LtAxillaPos.x())//select vertex and save vert-index _y axis
		{ 
			double smooth_elm = smoothingAxisX(RtAxillaPos.x(),LtAxillaPos.x(),a[i].x());	
			manCircumWeight[i] = smooth_elm*manCircumWeight(RtAnteriorMidaxillaPos.y(),RtLateralWaistNaturalIndentationPos.y(),a[i].y());//bust
			if(LtLateralWaistNaturalIndentationPos.z() > a[i].z())
			{
				real dist = LtLateralWaistNaturalIndentationPos.z() - a[i].z();
				manCircumWeight[i] = manCircumWeight[i]/(1+dist/7);	
			}
			tmp += manCircumWeight[i];				
		}
		else {
			manCircumWeight[i] = 0;
		}
		if(max < manCircumWeight[i])
			max = manCircumWeight[i];
	}
	for(int i = 0; i<a.Size();i++)
	{
		manCircumWeight[i] = 1.2*(manCircumWeight[i]/max);		
	}	
	circumGenRule->Genlist[1]->manCustomWeightBust.push_back(manCircumWeight);
}
void ModelBodyGenRules::waistWeightGen(){
	ArrayN<real> manCircumWeightU;
	ArrayN<real> manCircumWeightL;
	
	Vector3r RtLateralShoulderPos = circumGenRule->Genlist[1]->bms->BLs->blList[(circumGenRule->Genlist[1]->bms->BLs->GetLandmarkIdx("Rt_Lateral_Shoulder"))].pos; //x axis
	Vector3r LtLateralShoulderPos = circumGenRule->Genlist[1]->bms->BLs->blList[circumGenRule->Genlist[1]->bms->BLs->GetLandmarkIdx("Lt_Lateral_Shoulder")].pos; // x axis

	Vector3r RtInferiorBreastPos = circumGenRule->Genlist[1]->bms->BLs->blList[circumGenRule->Genlist[1]->bms->BLs->GetLandmarkIdx("Rt_Inferior_Breast")].pos; //y axis
	Vector3r AbdominalProtrusionPos =circumGenRule->Genlist[1]->bms->BLs->blList[circumGenRule->Genlist[1]->bms->BLs->GetLandmarkIdx("Abdominal_Protrusion")].pos; //y axis

	Vector3r LtLateralWaistNaturalIndentationPos = circumGenRule->Genlist[1]->bms->BLs->blList[circumGenRule->Genlist[1]->bms->BLs->GetLandmarkIdx("Lt_Lateral_Waist_natural_indentation")].pos; //z axis
	ArrayN<Vector3r,0> a=body->getBody()->riggingObject->get_posVert_base();
	
	manCircumWeightU.Resize(a.Size());
		
		int tmp = 0;
		double max = 0;
		for(int i = 0; i < a.Size(); i++){		
			if( RtInferiorBreastPos.y()>a[i].y() &&
				a[i].y()>AbdominalProtrusionPos.y() && 
				RtLateralShoulderPos.x()<a[i].x() && 
				a[i].x()<LtLateralShoulderPos.x()) 
				{
					manCircumWeightU[i] = manCircumWeight(RtInferiorBreastPos.y(),AbdominalProtrusionPos.y(),a[i].y());//bust
					if(LtLateralWaistNaturalIndentationPos.z() > a[i].z()){
						manCircumWeightU[i] = manCircumWeightU[i]*double(0.9);
					}
					tmp += manCircumWeightU[i];				
			}
			else {
				manCircumWeightU[i] = 0;
			}
			if(max < manCircumWeightU[i])
				max = manCircumWeightU[i];
		}
		for(int i = 0; i<a.Size();i++)
		{
			manCircumWeightU[i] = 0.3*(manCircumWeightU[i]/max);		
		}	
		circumGenRule->Genlist[2]->manCustomWeightWaist.push_back(manCircumWeightU);
		circumGenRule->Genlist[2]->manCustomWeightWaist.push_back(manCircumWeightU);
}
void ModelBodyGenRules::kneeWeightGen(){
	ArrayN<real> manCircumWeightL;
	ArrayN<real> manCircumWeightR;
	Vector3r RtMidpatellaPos = circumGenRule->Genlist[1]->bms->BLs->blList[(circumGenRule->Genlist[1]->bms->BLs->GetLandmarkIdx("Rt_Midpatella"))].pos; //x axis

	Vector3r RtMidthighPos = circumGenRule->Genlist[1]->bms->BLs->blList[circumGenRule->Genlist[1]->bms->BLs->GetLandmarkIdx("Rt_Midthigh")].pos; //y axis
	Vector3r RtMedialMalleousPos =circumGenRule->Genlist[1]->bms->BLs->blList[circumGenRule->Genlist[1]->bms->BLs->GetLandmarkIdx("Rt_Medial_Malleous")].pos; //y axis

	Vector3r crotchPos = circumGenRule->Genlist[1]->bms->BLs->blList[circumGenRule->Genlist[1]->bms->BLs->GetLandmarkIdx("Crotch")].pos; //center

	double upperPos = (2*RtMidpatellaPos.y() + RtMidthighPos.y())/3;
	double lowerPos = (3*RtMidpatellaPos.y() + RtMedialMalleousPos.y())/4;


	ArrayN<Vector3r,0> a=body->getBody()->riggingObject->get_posVert_base();
	
	manCircumWeightL.Resize(a.Size());
	manCircumWeightR.Resize(a.Size());
	

		double max = 0;
		double max_r = 0;
		for(int i = 0; i < a.Size(); i++){		
			if( upperPos>a[i].y() &&a[i].y()>lowerPos ) { 
				if( a[i].x() > crotchPos.x())
					manCircumWeightL[i] = manCircumWeight(upperPos,lowerPos,a[i].y());
				else
					manCircumWeightR[i] = manCircumWeight(upperPos,lowerPos,a[i].y());
			}
			else {
				manCircumWeightL[i] = 0;
				manCircumWeightR[i] = 0;
			}
			if(max < manCircumWeightL[i])
				max = manCircumWeightL[i];
			if(max_r < manCircumWeightR[i])
				max_r = manCircumWeightR[i];
		}
		for(int i = 0; i<a.Size();i++)
		{
			manCircumWeightL[i] = 1*(manCircumWeightL[i]/max);		
		}	
		for(int i = 0; i<a.Size();i++)
		{
			manCircumWeightR[i] = 1*(manCircumWeightR[i]/max_r);		
		}	
		circumGenRule->Genlist[5]->man_custom_weight_knee.push_back(manCircumWeightL);
		circumGenRule->Genlist[5]->man_custom_weight_knee.push_back(manCircumWeightR);

}