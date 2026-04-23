#pragma once

#include <rev/SparkBase.h>
#include <rev/REVLibError.h>
#include <functional>

namespace SparkUtil
{
    inline bool sparkStickyFault = false;

    static void ifOk(rev::spark::SparkBase &spark, std::function<double()> supplier, std::function<void(double)> consumer)
    {
        double value = supplier();
        if (spark.GetLastError() == rev::REVLibError::kOk)
        {
            consumer(value);
        }
        else
        {
            sparkStickyFault = true;
        }
    }

    static void ifOk(rev::spark::SparkBase &spark, std::vector<std::function<double()>> suppliers, std::function<void(std::vector<double>)> consumer)
    {
        std::vector<double> values(suppliers.size());
        for (int i = 0; i < suppliers.size(); ++i)
        {
            values[i] = suppliers[i]();
            if (spark.GetLastError() != rev::REVLibError::kOk)
            {
                sparkStickyFault = true;
                return;
            }
        }
        consumer(values);
    }

    static void tryUntilOk(int maxAttempts, std::function<rev::REVLibError()> command)
    {
        for (int i = 0; i < maxAttempts; ++i)
        {
            auto error = command();
            if (error == rev::REVLibError::kOk)
            {
                break;
            }
            else
            {
                sparkStickyFault = true;
            }
        }
    }
};

using namespace SparkUtil;