
/*
 * InitializationException.h
 *
 *  Created on: Apr 21, 2020
 */

#ifndef INITIALIZATONEXCEPTION_H_
#define INITIALIZATONEXCEPTION_H_

#include <stdexcept>

//https://riptutorial.com/cplusplus/example/23640/custom-exception
class InitializationException: virtual public std::runtime_error {

	protected:

		const char* method;

	public:

		/** Constructor (C++ STL string, int, int).
		 *  @param msg The error message
		 *  @param err_num Error number
		 *  @param err_off Error offset
		 */
		explicit InitializationException(const std::string& msg, const char* method): std::runtime_error(msg)	{
			InitializationException::method = method;
		}

		/** Destructor.
		 *  Virtual to allow for subclassing.
		 */
		virtual ~InitializationException() throw () {}

		const char* get_method() const throw() {
			return method;
		}
};

#endif /* INITIALIZATONEXCEPTION_H_ */
