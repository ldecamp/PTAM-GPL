#include <vector>
#include <string>

struct StatsData{
	double Scale;
	std::vector<std::vector<double> > PoseEstimates; //Ordered from first to last (X,Y,Z,D) D=scene depth
	int MaxKeyFramesLost;
	inline bool HasLostFrames() { return MaxKeyFramesLost>0; }
};

class Stats{
public:
	static void SetScale(double scale); 
	static void SetPosesEstimates(std::vector<std::vector<double> > estimates);
	static void SetFramesLost(int framesLost);

	static void SaveStats();

protected:
	static void SaveEstimates(std::string filepath);
	static void SaveInfos(std::string filepath);
	static StatsData mbData;
	
};
