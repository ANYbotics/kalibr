#ifndef SM_LOGGING_STDOUT_LOGGER_HPP
#define SM_LOGGING_STDOUT_LOGGER_HPP

#include <sm_logging/logging/Logger.hpp>
#include <sm_logging/logging/Formatter.hpp>

namespace sm {
    namespace logging {
        
        class StdOutLogger : public Logger
        {
        public:
            StdOutLogger();
            virtual ~StdOutLogger();
            
            Formatter formatter;
        protected:
            virtual void logImplementation(const LoggingEvent & event);
        };

    } // namespace logger
} // namespace sm


#endif /* SM_LOGGING_STDOUT_LOGGER_HPP */
