#include "Statistics.h"
#include <gvars3/instances.h>
#include <iostream>
#include <fstream>

using namespace std;
using namespace GVars3;

StatsData Stats::mbData;

//Update Stats Container
void Stats::SetScale(double scale){
	mbData.Scale = scale;
}

void Stats::SetPosesEstimates(std::vector<std::vector<double> > estimates){
	mbData.PoseEstimates = estimates;
}

void Stats::SetFramesLost(int framesLost){
	if(mbData.MaxKeyFramesLost < framesLost)
		mbData.MaxKeyFramesLost = framesLost;
}
//End update stats container 

//Save post estimates to csv file
void Stats::SaveEstimates(std::string filepath){
	ofstream outputFile;
	outputFile.open(filepath.c_str(), std::ofstream::out | std::ofstream::trunc);
	outputFile << "KFID,X,Y,Z,Depth" << std::endl;
	for(unsigned int i=0;i<mbData.PoseEstimates.size();i++){
		std::vector<double> pose=mbData.PoseEstimates[i];
		outputFile << pose[0] << ",";
		outputFile << pose[1] << ",";
		outputFile << pose[2] << ",";
		outputFile << pose[3] << ",";
		outputFile << pose[4] << std::endl;
	}
	outputFile.close();
}

//Save General info to csv file (Scene depth + Keyframe lost)
void Stats::SaveInfos(std::string filepath){
	ofstream outputFile;
	outputFile.open(filepath.c_str(), std::ofstream::out | std::ofstream::trunc);
	outputFile << "Scale,NFramesLost" << std::endl; 
	outputFile << mbData.Scale << ",";
	outputFile << mbData.MaxKeyFramesLost << std::endl;
	outputFile.close();
}


//Warning: do not automatically create directory
const string STATS_DEFAULT_OUTPOUT = "./Statistics"; 
// Save All stats to csv file
void Stats::SaveStats(){    
    static gvar3<std::string> mgvdStatsOutputDir("Stats.OutputDirectory", STATS_DEFAULT_OUTPOUT, SILENT);
	SaveEstimates(*mgvdStatsOutputDir+"/poseEstimates.csv");
	SaveInfos(*mgvdStatsOutputDir+"/infos.csv");
}