
#pragma once

#include <plog/Log.h>
#include <plog/Initializers/RollingFileInitializer.h>
#include <plog/Record.h>
#include <plog/Util.h>
#include <iomanip>

namespace Logging {
    // Define log IDs for subsystems.
    enum Loggers {
        Default = 0,
        SwerveDrive = 1,
        Arm = 2
    };
} // namespace Logging

namespace plog
{
    class OKCFormatter
    {
    public:
        static util::nstring header() 
        {
            return PLOG_NSTR("Date,Time,Severity,TID,This,Function,Label,Data\n");
        }
        
        static util::nstring format(const Record& record)
        {
            tm t;
            util::localtime_s(&t, &record.getTime().time);

            util::nostringstream ss;
            ss << t.tm_year + 1900 << PLOG_NSTR("/") << std::setfill(PLOG_NSTR('0')) << std::setw(2) << t.tm_mon + 1 << PLOG_NSTR("/") << std::setfill(PLOG_NSTR('0')) << std::setw(2) << t.tm_mday << PLOG_NSTR(",");
            ss << std::setfill(PLOG_NSTR('0')) << std::setw(2) << t.tm_hour << PLOG_NSTR(":") << std::setfill(PLOG_NSTR('0')) << std::setw(2) << t.tm_min << PLOG_NSTR(":") << std::setfill(PLOG_NSTR('0')) << std::setw(2) << t.tm_sec << PLOG_NSTR(".") << std::setfill(PLOG_NSTR('0')) << std::setw(3) << static_cast<int> (record.getTime().millitm) << PLOG_NSTR(",");
            ss << severityToString(record.getSeverity()) << PLOG_NSTR(",");
            ss << record.getTid() << PLOG_NSTR(",");
            ss << record.getObject() << PLOG_NSTR(",");
            ss << record.getFunc() << PLOG_NSTR("@") << record.getLine() << PLOG_NSTR(",");

            util::nstring message = record.getMessage();

            // Split the message into label and data.
            size_t comma_idx = message.find(',');

            util::nstring label = "/none";
            util::nstring data = message;
            if (comma_idx != util::nstring::npos)
            {
                label = message.substr(0, comma_idx);
                data = message.substr(comma_idx + 1, message.size() - (comma_idx + 1));
            } else {
                // Use the default values defined above
            }
            
            // Print the label and data to the log
            ss << label << PLOG_NSTR(",");
            ss << data;

            ss << PLOG_NSTR("\n");

            return ss.str();
        }
    };
}