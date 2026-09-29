#pragma once

/**
 * @class Command
 * @brief Base command class
 */
class Command {
public:
    virtual ~Command() = default;

    /**
     * @class Result
     * @brief Return value for exec method.
     */
    enum class Result {
        Complete,
        Error,
        Running,
        Failure
    };

    virtual Result exec() = 0;
};