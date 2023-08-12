/**
 * @file   git_version.hpp
 * @author Dominik Authaler
 * @date   12.08.2023
 *
 * Utility file for including the hash of the latest commit within the executable.
 * Based on https://jonathanhamberg.com/post/cmake-embedding-git-hash/.
 */

#ifndef RIG_RECONFIGURE_GIT_VERSION_HPP
#define RIG_RECONFIGURE_GIT_VERSION_HPP

extern const char *gitHash;

#endif //RIG_RECONFIGURE_GIT_VERSION_HPP
