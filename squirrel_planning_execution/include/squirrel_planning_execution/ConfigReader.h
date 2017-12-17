#ifndef SQUIRREL_PLANNING_EXECUTABLE_CONFIG_READER_H
#define SQUIRREL_PLANNING_EXECUTABLE_CONFIG_READER_H

#include <ros/ros.h>
#include <mongodb_store/message_store.h>
#include <geometry_msgs/Pose.h>
#include <boost/concept_check.hpp>

namespace KCL_rosplan
{

/**
 * Utility class to read a configutation file and store the results in the mongodb message store.
 */
class ConfigReader
{
public:
	/**
	 * Constructor.
	 * @param nh The node handle.
	 */
	ConfigReader(ros::NodeHandle &nh);
	
	/**
	 * Read a configuration file and store the result into the given knowledge base.
	 * @param config_file The file to parse.
	 * @param ms The MongoDB database to store information in.
	 * @return True if the file was parsed successful, false otherwise.
	 */
	bool readConfigurationFile(const std::string& config_file, mongodb_store::MessageStoreProxy& ms);
	
	/**
	 * Add a fact to the knowledge base.
	 * @param predicate The predicate to add.
	 * @param parameters The parameters of the fact.
	 * @param is_negative Whether the fact added is true or false.
	 * @return True if the fact was added, false otherwise.
	 */
	bool addFact(const std::string& predicate, const std::map<std::string, std::string>& parameters, bool is_negative);
	
	/**
	 * Add an instance to the knowledge base.
	 * @param type The predicate to add.
	 * @param name The parameters of the fact.
	 * @return True if the fact was added, false otherwise.
	 */
	bool addInstance(const std::string& type, const std::string& name);
	
	/**
	 * Add a function to the knowledge base.
	 * @param function The predicate to add.
	 * @param parameters The parameters of the fact.
	 * @param value The value of the function.
	 * @return True if the function was added, false otherwise.
	 */
	bool addFunction(const std::string& function, const std::map<std::string, std::string>& parameters, float value);
	
private:
	
	/**
	 * Process a box and store it in the knowledge base.
	 * @param tokens The read tokens.
	 * @param ms The MongoDB database to store information in.
	 * @param line The read line.
	 * @return True if the box was parsed successful, false otherwise.
	 */
	bool processBox(const std::vector<std::string>& tokens, mongodb_store::MessageStoreProxy& ms, const std::string& line);
	
	/**
	 * Process a toy and store it in the knowledge base.
	 * @param tokens The read tokens.
	 * @param ms The MongoDB database to store information in.
	 * @param line The read line.
	 * @return True if the toy was parsed successful, false otherwise.
	 */
	bool processToy(const std::vector<std::string>& tokens, mongodb_store::MessageStoreProxy& ms, const std::string& line);
	
	/**
	 * Process a waypoint and store it in the knowledge base.
	 * @param tokens The read tokens.
	 * @param ms The MongoDB database to store information in.
	 * @param line The read line.
	 * @return True if the waypoint was parsed successful, false otherwise.
	 */
	bool processWaypoint(const std::vector<std::string>& tokens, mongodb_store::MessageStoreProxy& ms, const std::string& line);
	
	/**
	 * Process a child and store it in the knowledge base.
	 * @param tokens The read tokens.
	 * @param ms The MongoDB database to store information in.
	 * @param line The read line.
	 * @return True if the child was parsed successful, false otherwise.
	 */
	bool processChild(const std::vector<std::string>& tokens, mongodb_store::MessageStoreProxy& ms, const std::string& line);
	
	/**
	 * Process a object to box mapping and store it in the knowledge base.
	 * @param tokens The read tokens.
	 * @param ms The MongoDB database to store information in.
	 * @param line The read line.
	 * @return True if the mapping was parsed successful, false otherwise.
	 */
	bool processObjectToBoxMapping(const std::vector<std::string>& tokens, mongodb_store::MessageStoreProxy& ms, const std::string& line);
	
	/**
	 * Process a function.
	 * @param tokens The read tokens.
	 * @param ms The MongoDB database to store information in.
	 * @param line The read line.
	 * @return True if the function parsed successful, false otherwise.
	 */
	bool processFunction(const std::vector<std::string>& tokens, mongodb_store::MessageStoreProxy& ms, const std::string& line);
	
	/**
	 * Send a marker to rviz for debugging.
	 * @param The pose to send a marker.
	 * @param name The name of the marker.
	 * @param size The size of the marker.
	 */
	void sendMarker(const geometry_msgs::Pose& pose, const std::string& name, float size);
	
	/**
	 * Tokenise the string 's' and return a list of seperate words.
	 * @param s The string to be tokenised.
	 * @param tokens All found words are stored in this list.
	 */
	void tokenise(const std::string& s, std::vector<std::string>& tokens);
	
	/**
	 * Transform a string to a pose, the expected format is f,f,f.
	 * @param s The string to transform to a pose.
	 */
	geometry_msgs::Pose transformToPose(const std::string& s);
	
	ros::NodeHandle* node_handle;
	
	// knowledge service clients.
	ros::ServiceClient update_knowledge_client;
	ros::ServiceClient get_instance_client;
	ros::ServiceClient get_attribute_client;
	ros::ServiceClient query_knowledge_client;
	
	// Publisher to the visualiser.
	ros::Publisher vis_pub;
};

};

#endif
