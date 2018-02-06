#ifndef MAPPING_H_
#define MAPPING_H_

#include "types.h"

void vMainMappingTask( void *pvParameters );

/**
 * @brief      Updates each of the point buffers corresponding to each distance
 *             sensor with a position calculated from the given measurement data
 *             and the robot's current pose.
 *
 * @param      Buffers      A pointer to the array of point buffers
 * @param[in]  Measurement  A struct containing measurement data
 * @param[in]  Pose         The current pose of the robot
 */
static void mapping_update_point_buffers(point_buffer_t *Buffers, measurement_t Measurement, pose_t Pose);

/**
 * @brief      Checks the points in the point buffers for collinearity, and adds
 *             the extracted line segments to the line buffer.
 *
 * @param      PointBuffer  A pointer to a point buffer
 * @param      LineBuffer   A pointer to a line buffer
 */
static void mapping_line_create(point_buffer_t *PointBuffer, line_buffer_t *LineBuffer);

/**
 * @brief      Compare all the lines in the LineBuffer with the ones in
 *             LineRepo, and replaces the ones in LineRepo with the ones that
 *             can be merged. If a line cannot be merged it is added to the end
 *             of the LineRepo.
 *
 * @param      LineBuffer  A reference to a LineBuffer
 * @param      LineRepo    A reference to the LineRepo
 */
static void mapping_line_merge(line_buffer_t *LineBuffer, line_repo_t *LineRepo);

/**
 * @brief      Determine if 2 lines are eligible for merging.
 *
 * @param      Line1  The line 1
 * @param      Line2  The line 2
 *
 * @return     { 1 if lines are mergeable, 0 if not }
 */
static uint8_t mapping_is_mergeable(line_t *Line1, line_t *Line2);

/**
 * @brief      Merges two lines and returns a new line from the calculated
 *             parameters.
 *
 * @param      Line1  The line 1
 * @param      Line2  The line 2
 *
 * @return     { line_t }
 */
static line_t mapping_merge_segments(line_t *Line1, line_t *Line2);

static line_repo_t* mapping_repo_merge(line_buffer_t *Repo);

/**
 * @brief      Check if 3 points are collinear within a given tolerance
 *
 * @param      a     { parameter_description }
 * @param      b     { parameter_description }
 * @param      c     { parameter_description }
 *
 * @return     { 1 if the points are collinear }
 */
static uint8_t mapping_are_collinear(point_t *a, point_t *b, point_t *c);

static uint8_t mapping_is_empty(line_t line);

#endif