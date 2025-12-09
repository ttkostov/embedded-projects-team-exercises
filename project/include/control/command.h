#pragma once
#include <Arduino.h>

struct CommandParseResult
{
  static constexpr uint8_t MAX_FLAGS = 6;
  static constexpr uint8_t MAX_PARAMS = 6;

  String command;         // e.g. "list"
  String rawValue;        // e.g. "/tes/path"
  float numericValue = 0; // parsed number if present

  String flags[MAX_FLAGS]; // e.g. "--recursive"
  uint8_t flagCount = 0;

  String paramKeys[MAX_PARAMS];   // e.g. "key"
  String paramValues[MAX_PARAMS]; // e.g. "value"
  uint8_t paramCount = 0;

  bool hasFlag(const char *flag) const
  {
    for (uint8_t i = 0; i < flagCount; i++)
      if (flags[i].equalsIgnoreCase(flag))
        return true;
    return false;
  }

  bool hasParam(const char *key) const
  {
    for (uint8_t i = 0; i < paramCount; i++)
      if (paramKeys[i].equalsIgnoreCase(key))
        return true;
    return false;
  }

  float getParamFloat(const char *key, float defaultVal = 0) const
  {
    for (uint8_t i = 0; i < paramCount; i++)
      if (paramKeys[i].equalsIgnoreCase(key))
        return paramValues[i].toFloat();
    return defaultVal;
  }
};

class CommandParser
{
public:
  CommandParseResult parse(const String &msg)
  {
    CommandParseResult result;

    int colon = msg.indexOf(':');
    if (colon < 0)
      return result;

    result.command = msg.substring(0, colon);
    String remainder = msg.substring(colon + 1);
    remainder.trim();

    tokenize(result, remainder);
    extractNumber(result);

    return result;
  }

private:
  void tokenize(CommandParseResult &out, const String &input)
  {
    unsigned int start = 0;

    while (start < input.length())
    {
      int end = input.indexOf(' ', start);
      if (end < 0)
        end = input.length();

      String tok = input.substring(start, end);
      tok.trim();

      if (tok.length() > 0)
        processToken(out, tok);

      start = end + 1;
    }
  }

  void processToken(CommandParseResult &out, const String &tok)
  {
    // Parameter: --key=value
    if (tok.startsWith("--") && tok.indexOf('=') > 0)
    {
      int eq = tok.indexOf('=');
      if (out.paramCount < out.MAX_PARAMS)
      {
        out.paramKeys[out.paramCount] = tok.substring(2, eq);
        out.paramValues[out.paramCount] = tok.substring(eq + 1);
        out.paramCount++;
      }
      return;
    }

    // Flag: --option
    if (tok.startsWith("--"))
    {
      if (out.flagCount < out.MAX_FLAGS)
        out.flags[out.flagCount++] = tok;
      return;
    }

    // Otherwise: it's the main value
    if (out.rawValue.length() == 0)
      out.rawValue = tok;
  }

  void extractNumber(CommandParseResult &out)
  {
    if (out.rawValue.length() > 0)
      out.numericValue = out.rawValue.toFloat();
  }
};
