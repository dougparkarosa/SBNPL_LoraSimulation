#ifndef DEBUGBREAK_H
#define DEBUGBREAK_H

inline void DebugBreak() {
    asm("int $3");
}

#endif // DEBUGBREAK_H