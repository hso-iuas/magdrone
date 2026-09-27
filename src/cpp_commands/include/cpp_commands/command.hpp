#pragma once

class Command {
public:
    virtual ~Command() = default;

    enum class Result {
        Complete,
        Error,
        Running,
        Failure
    };

    virtual Result exec() = 0;
};