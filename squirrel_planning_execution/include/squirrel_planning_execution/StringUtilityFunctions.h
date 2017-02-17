#include <map>
#include <vector>
#include <boost/concept_check.hpp>

#include <algorithm> 
#include <functional> 
#include <cctype>
#include <locale>

#ifndef SQUIRREL_PLANNING_EXECUTION_STRING_UTILITY_FUNCTIONS_H
#define SQUIRREL_PLANNING_EXECUTION_STRING_UTILITY_FUNCTIONS_H

/**
 * An utility class to perform common string operations.
 */
namespace KCL_rosplan {
	

	/**
	 * trim from start (in place)
	 */
	static inline void ltrim(std::string &s)
	{
		s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
	}

	/**
	 * trim from end (in place)
	 */
	static inline void rtrim(std::string &s)
	{
		s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
	}

	/**
	 * trim from both ends (in place)
	 */
	static inline void trim(std::string &s)
	{
		ltrim(s);
		rtrim(s);
	}

	/**
	 * trim from start (copying)
	 */
	static inline std::string ltrimCopy(const std::string& s)
	{
		std::string s_copy = s;
		ltrim(s_copy);
		return s_copy;
	}

	/**
	 * trim from end (copying)
	 */
	static inline std::string rtrimCopy(const std::string& s)
	{
		std::string s_copy = s;
		rtrim(s_copy);
		return s_copy;
	}

	/**
	 * trim from both ends (copying)
	 */
	static inline std::string trimCopy(const std::string& s)
	{
		std::string s_copy = s;
		trim(s_copy);
		return s_copy;
	}
	
	/**
	 * Tokenize a string.
	 */
	static std::vector<std::string> split(const std::string& s, char c = ' ')
	{
		std::vector<std::string> result;
		const char* str = s.c_str();
		do
		{
			const char* begin = str;

			while(*str != c && *str)
			{
					str++;
			}

			result.push_back(std::string(begin, str));
		} while (0 != *str++);

		return result;
	}
}

#endif
