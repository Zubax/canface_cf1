/*
 * Copyright (c) 2016 Zubax, zubax.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

/*
 * Simple command shell designed as a more versatile replacement to the ChibiOS embedded shell.
 */

#pragma once

#include <ch.hpp>
#include <chprintf.h>
#include <memstreams.h>
#include <cstring>
#include <cstdint>
#include <cstdarg>
#include <cassert>
#include <zubax_chibios/os.hpp>

namespace shell
{
/**
 * This class wraps the underlying OS primitive used to access the shell communication channel.
 */
class BaseChannelWrapper
{
    BaseChannel* channel_;

    std::size_t writeExpandingCrLf(unsigned character_timeout_msec, const char* str)
    {
        std::size_t ret = 0;

        for (const char* pc = str; *pc != '\0'; pc++)
        {
            if (*pc == '\n')
            {
                if (MSG_OK != chnPutTimeout(channel_, '\r', MS2ST(character_timeout_msec)))
                {
                    break;
                }
                ret++;
            }
            if (MSG_OK != chnPutTimeout(channel_, *pc, MS2ST(character_timeout_msec)))
            {
                break;
            }
            ret++;
        }

        return ret;
    }

public:
    static constexpr unsigned DefaultWriteCharacterTimeoutMSec = 2;

    BaseChannelWrapper(BaseChannel* chan) :
        channel_(chan)
    { }

    BaseChannel* getChannel()  const { return channel_; }
    void setChannel(BaseChannel* chan) { channel_ = chan; }

    template <unsigned BufferSize = 200, unsigned CharacterTimeoutMSec = DefaultWriteCharacterTimeoutMSec>
    std::size_t vprint(const char* format, va_list vl)
    {
        MemoryStream ms;
        char buffer[BufferSize];
        msObjectInit(&ms, reinterpret_cast<std::uint8_t*>(buffer), BufferSize, 0);

        auto chp = reinterpret_cast<::BaseSequentialStream*>(&ms);
        chvprintf(chp, format, vl);

        chSequentialStreamPut(chp, 0);

        return writeExpandingCrLf(CharacterTimeoutMSec, buffer);
    }

    template <typename... ReferToVPrintForArgs>
    std::size_t print(const char* format, ...)
    {
        va_list vl;
        va_start(vl, format);
        const auto ret = vprint<ReferToVPrintForArgs...>(format, vl);
        va_end(vl);
        return ret;
    }

    int getChar(unsigned timeout_msec)
    {
        return chnGetTimeout(channel_, MS2ST(timeout_msec));
    }

    int putChar(char chr, unsigned timeout_msec = DefaultWriteCharacterTimeoutMSec)
    {
        return chnPutTimeout(channel_, chr, MS2ST(timeout_msec));
    }

    // More methods may be added in the future
};

/**
 * Inherit this interface to implement a command handler.
 */
class ICommandHandler
{
    ICommandHandler(const ICommandHandler&) = delete;
    ICommandHandler& operator=(const ICommandHandler&) = delete;

protected:
    ICommandHandler() { }

public:
    virtual ~ICommandHandler() { }

    virtual const char* getName() const = 0;

    virtual void execute(BaseChannelWrapper& ios, int argc, char** argv) = 0;
};

/**
 * Implementation details, do not use directly.
 */
namespace impl_
{

class Tokenizer         // A la strtok()
{
    char* token_ptr_ = nullptr;

public:
    char* tokenize(char* str)
    {
        if (str)
        {
            token_ptr_ = str;
        }
        char* token = token_ptr_;
        if (!token)
        {
            return NULL;
        }
        static const char* const Delims = " \t";
        token += std::strspn(token, Delims);
        token_ptr_ = std::strpbrk(token, Delims);
        if (token_ptr_)
        {
            *token_ptr_++ = '\0';
        }
        return *token ? token : NULL;
    }
};

class HelpCommandHandler : public ICommandHandler
{
    const ICommandHandler* const* const command_handlers_;
    const unsigned max_handlers_;

    const char* getName() const override { return "help"; }

    void execute(BaseChannelWrapper& ios, int, char**) override
    {
        ios.print("Available commands:\n");
        auto x = command_handlers_;
        for (unsigned i = 0; i < max_handlers_; i++)
        {
            if (*x != nullptr)
            {
                ios.print("\t%s\n", (*x)->getName());
            }
            x++;
        }
    }

public:
    HelpCommandHandler(const ICommandHandler* const* handlers, unsigned max_handlers) :
        command_handlers_(handlers),
        max_handlers_(max_handlers)
    { }
};

}

/**
 * Operating mode of the shell.
 */
enum class Mode
{
    Normal,     //!< Normal mode, regular shell behavior
    Silent      //!< No echo unless the command is recognized; no prompt. Useful for bootloaders etc.
};

/**
 * Shell command processor class.
 */
template <unsigned MaxCommandHandlers = 10,
          unsigned MaxLineLength = 200,
          unsigned MaxCommandArguments = 8>
class Shell
{
    ICommandHandler* command_handlers_[MaxCommandHandlers] = {};

    char line_buffer_[MaxLineLength + 1] = {};
    unsigned pos_ = 0;

    bool need_prompt_ = true;
    Mode mode_;

    impl_::HelpCommandHandler help_command_handler_;

    void echo(BaseChannelWrapper& ios, char chr) const
    {
        if (mode_ != Mode::Silent)
        {
            (void)ios.putChar(chr);
        }
    }

    void processCommand(BaseChannelWrapper& ios, const char* line) const
    {
        // Creating a mutable copy of the line, as the original needs to be retained for silent mode
        char mutable_copy_of_line[MaxLineLength + 1];
        std::strncpy(mutable_copy_of_line, line, sizeof(mutable_copy_of_line));

        // Tokenizing
        char* argv[MaxCommandArguments] = {};
        std::uint8_t argc = 0;
        impl_::Tokenizer tokenizer;
        {
            char* token = tokenizer.tokenize(mutable_copy_of_line);
            while (token != nullptr)
            {
                argv[argc++] = token;
                if (argc >= MaxCommandArguments)
                {
                    break;
                }
                token = tokenizer.tokenize(nullptr);
            }
        }

        if (argc == 0)
        {
            return;     // No command provided, ignore
        }

        // Command lookup, exit on success
        for (auto x : command_handlers_)
        {
            if ((x != nullptr) && (std::strcmp(x->getName(), argv[0]) == 0))
            {
                // In silent mode we only echo if the command is recognized
                if (mode_ == Mode::Silent)
                {
                    (void)ios.print("%s\n", line);
                }
                x->execute(ios, argc, argv);
                return;
            }
        }

        // No such command
        if (mode_ != Mode::Silent)
        {
            (void)ios.print("! ERROR: \"%s\"?\n", argv[0]);
        }
    }

public:
    Shell(Mode mode = Mode::Normal) :
        mode_(mode),
        help_command_handler_(command_handlers_, MaxCommandHandlers)
    {
        addCommandHandler(&help_command_handler_);
    }

    Mode getMode()    const { return mode_; }
    void setMode(Mode mode) { mode_ = mode; }

    bool addCommandHandler(ICommandHandler* chr)
    {
        for (auto& x : command_handlers_)
        {
            if (x == nullptr)
            {
                x = chr;
                return true;
            }
        }
        return false;
    }

    void runFor(BaseChannelWrapper& ios, unsigned run_duration_msec)
    {
        const auto run_duration_st = MS2ST(run_duration_msec);
        const auto started_at_st = chVTGetSystemTime();

        do
        {
            if (need_prompt_)
            {
                need_prompt_ = false;
                if (mode_ != Mode::Silent)
                {
                    (void)ios.print("> ");
                }
            }

            // Reading new character
            const auto elapsed = chVTTimeElapsedSinceX(started_at_st);
            const auto read_timeout_st = (run_duration_st > elapsed) ? (run_duration_st - elapsed) : 1;
            const auto chr = ios.getChar(std::max(1U, unsigned(ST2MS(read_timeout_st))));
            if (chr < 0)
            {
                continue;
            }

            // Processing the character
            if (chr == '\r')                            // End of command
            {
                echo(ios, '\r');
                echo(ios, '\n');
                line_buffer_[pos_] = '\0';
                if (pos_ > 0)
                {
                    processCommand(ios, line_buffer_);
                }
                reset();
            }
            else if (chr == 8 || chr == 127)            // DEL or BS (backspace)
            {
                if (pos_ > 0)
                {
                    echo(ios, 8);       // Erase last char and move caret back
                    echo(ios, ' ');     // Put space on top of the erased character
                    echo(ios, 8);       // Move the caret back again
                    pos_ -= 1;
                }
            }
            else if (chr >= 32)                         // Normal printable ASCII character and everything above ASCII
            {
                if (pos_ < MaxLineLength)
                {
                    echo(ios, chr);
                    line_buffer_[pos_++] = char(chr);
                }
            }
            else                                        // This also includes Ctrl+C, Ctrl+D, and LF
            {
                ;                                       // Invalid byte - ignore
            }

            // Invariants
            assert(pos_ <= MaxLineLength);
        }
        while (chVTTimeElapsedSinceX(started_at_st) <= run_duration_st);
    }

    void reset()
    {
        assert(pos_ <= MaxLineLength);
        pos_ = 0;
        need_prompt_ = true;
    }
};

}
