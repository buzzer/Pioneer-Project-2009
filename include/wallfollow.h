/// @file wallfollowing.h
/// @author Sebastian Rockel
///
/// @mainpage Robotic Project 2009
///
/// @par Copyright
/// Copyright (C) 2009 Sebastian Rockel.
/// This program can be distributed and modified under the condition mentioning
/// the @ref author.
///
/// @par Description
/// Some global definitions here
///
const double TRACKING_NO = 10 * M_PI;///< Disable camera tracking

typedef struct ts_Ball {
  int num;
  double dist;
  double angle;
};
