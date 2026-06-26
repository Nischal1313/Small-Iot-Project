# Instructions for AI

## Project Summary

- **Language**: C++ / C
- **Formatter:** `clang-format` see `.clang-format`

---

## General Approach

Be systematic: read all rules first, create a checklist, and methodically apply each rule to ensure nothing is missed.

---

## Memory & Ownership

- **Strict RAII:** No raw `new` or `delete`.
- Use `std::unique_ptr` for exclusive ownership.
- Use `std::shared_ptr` only when shared ownership is strictly required.

---

## Naming Conventions

- **CS-002** Class and struct data member names are terminated with the letter `M`.
- **CS-003** Function parameter names are terminated with the letter `P`.
- **CS-004** Static variable names are terminated with the letter `S`.
- **CS-005** Global variable names are terminated with the letter `G`.
- **CS-006** Do not prefix enum values with `k`.
- **CS-009** Function names and parameter names must be camel case.
- **CS-010** Variable names must be camel case.
- **CS-011** Pointer type variables (raw or smart) must be prefixed with `p`.
- **CS-012** Reference type variables must be prefixed with `r`.
- **CS-013** Constant names must be uppercase snake case and not prefixed with `k`.
- **CS-014** Camel case naming applies to:
  - Class, struct, and union: data members, method names, method parameters.
  - Global variables, static variables, and local variables.
  - Free function names.
- Write the full names of variables, if the length is no longer than 15 characters.

---

## Code Formatting & Structure

- **CS-001** The opening bracket of a class, struct, or enum declaration starts on the next line.
- **CS-007** A function's opening bracket starts on the next line. Applies to inline methods and functions.
- **CS-008** Opening bracket starts on the next line for: `if`, `else`, `while`, `for`, `switch`.
- **CS-016** Indentation size is four space characters.
- **CS-017** Source file extension is `.cpp`.
- **CS-018** Use `#pragma once` instead of `#define` include guards.
- **CS-021** Place the `const` keyword after the object it describes:
  ```cpp
  int const* pInt; // pointer to a constant int
  ```
- **CS-029** Use uniform initializers for variables and data members.
- 

---

## Includes & Namespaces

- **CS-019** For global functions and variables, use an anonymous namespace.
- **CS-028** Use double quotes (`""`) for `#include` only when the header is in the same local folder as the including file. Use angle brackets (`<>`) when referencing a header relative to a parent or project include directory (e.g., `<message/request/request_message_base.h>`).

---

## Comments & Documentation
- In `.cpp` files use `//`. Add concise inline comments to explain intent, e.g. `enables BLE connection`, `needed for parsing`. 

- Comments should not include any unnecessary symbols. 

- For larger explanations, follow the Doxygen documentation guide. @details should include details only about the function, not of the project.

- Write self-documenting code that clearly describes what it does through naming and structure. Comments should explain why, not what — the code itself should make the "what" obvious.

---

## Constants & Magic Numbers

- **CS-013** Constant names must be uppercase snake case.

- Use constexpr variables or strongly typed enums for all constants. No magic numbers.
- Use fixed-width integer types (`int32_t`, `uint64_t`, etc.) to ensure explicit size and signedness when possible.
- Everything should be `const` or `constexpr` by default unless there is a reason for mutation. This principle applies to variables, functions and function parameters.
---

## Function Design

- **CS-025** Functions must have only one exit point. Store the result in a variable named `result` and return it at the end.
- **CS-026** If a function body has no branching instructions (`if`, `switch`, etc.), the `return` may directly return the expression (including ternary expressions).
- **Guard clauses (return early):** Validate preconditions at the top of a function. Return or throw immediately on failure, keeping the happy path at the end and preventing deep nesting. Use assertions.
- Every function must validate its parameters.
- Each function must contain at least one runtime assertion to catch unexpected conditions and document assumptions.
- Functions *must* be minimally nested — no more than 3–4 levels deep.
- A single function must not exceed 60–70 lines of logical code, unless it consists primarily of `switch`/`case` logic.
- Every non-void function's return value must be examined by its caller.
- Limit variable scope to be as local as possible.
- Write general-purpose, reusable functions rather than project-specific ones.
- Functions should accept project-specific values as parameters rather than hardcoding them

- Project-specific configuration should be passed in at call time, not embedded in the function definition. The goal is to have SRP and reuseable templates.
---

## Class Design

- **CS-020** Friends are allowed only in tests, not in production code.
- **Single-Responsibility Principle (SRP):** A class should have only *one* reason to change.
- **DRY:** Both SRP and DRY are tools for achieving maintainability and simplifying change — not goals in themselves. If separation produces adverse effects, do not implement it.
- A single source file must not exceed 500 lines of code. Divide and subdivide classes and functions to stay within this limit.
- Make invalid states impossible to represent. Always have a clear, well-defined boundary between valid and invalid states and nest state checks appropriately.

---

## Design Principles

- **CS-030** Apply SOLID and RAII principles when designing solutions.
- **SRP** applies to both classes, files and functions.
- Always have an upper bound for any loop or runtime-bounded iteration (including `while` loops and message queues) to prevent infinite loops and runtime failures.
- When parallel task execution is possible, always ask whether it should be done.
- Complexity is anything related to the structure of a software system that makes it hard to understand and modify the system. Minimize complexity based on this understanding of complexity.

---

## Refactoring Rules

- **CS-022** When renaming variables, do not rename function names that contain the same word.
- **CS-023** When renaming functions, do not rename variable names that contain the same word.
- **CS-015 / CS-024** Do not apply these coding guidelines to third-party source code. Do not rename third-party library variables or functions.

---

## Agent Behaviour

### Clarify Before Acting
- If multiple interpretations exist, present them — don't pick silently.
- If something is unclear, stop. Name what's confusing. Ask.
- Clarifying questions come before implementation, not after mistakes.

### Simplicity First
- Write the minimum code that solves the problem. Nothing speculative.
- No features beyond what was asked.
- No abstractions for single-use code.
- No error handling for impossible scenarios.
- If you write 200 lines and it could be 50, rewrite it.
- If a simpler approach exists, say so. Push back when warranted.

Ask yourself: *"Would a senior engineer say this is overcomplicated?"* If yes, simplify.

### Cleanup After Changes
When your changes create orphans:
- Remove imports, variables, and functions that **your** changes made unused.
- Do not remove pre-existing dead code unless explicitly asked.
- No need to build and test, I will provide the feedback.

### Success Criteria
These guidelines are working if:
- Diffs contain fewer unnecessary changes.
- Rewrites due to overcomplication are rare.
- Clarifying questions come before implementation, not after mistakes.
