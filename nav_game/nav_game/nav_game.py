#!/usr/bin/env python3
"""
nav_game.py

A ROS 2 “Jazzy”-style action server node. When launched, it:
  • Brings up a barebones pygame window (8×8 grid by default).
  • Waits for a Waypoints action goal (a list of geometry_msgs/Point).
  • For each Point in the goal, it tries to move “straight‐line” (no turning) from its current grid cell to that waypoint:
      – If aligned in the same row OR same column, it steps one cell at a time toward the waypoint.
      – If it ever hits a black‐obstacle cell, it aborts further waypoints, resets back to START_CELL, and returns the list of waypoints reached so far.
      – If it reaches the waypoint successfully, it appends it to the “successful_waypoints” list and moves on to the next.
      – If a waypoint is NOT straight‐aligned, it is treated as a violation: abort, reset to START_CELL, and return.
  • While moving, it publishes feedback (the complete trajectory so far, as geometry_msgs/Point[]).
  • After any violation, the game resets to START_CELL and awaits a new goal.
  • When a run completes (either due to violation or full success), it returns the list of successfully reached waypoints.
  • The red goal square is always drawn at a single static location (STATIC_GOAL).
  • **Now also redraws the player immediately after each step, so you can see the circle move in real time.**
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from nav_game_msgs.action import Waypoints
from geometry_msgs.msg import Point

import pygame
import time
import threading  # For Lock

# -------------------------------
# Configuration (change these)
# -------------------------------
GRID_WIDTH  = 8   # number of columns
GRID_HEIGHT = 8   # number of rows
CELL_SIZE   = 60  # pixel size of each cell in pygame window

# 0 = free (white), 1 = obstacle (black). Modify as needed.
# Must be GRID_HEIGHT rows × GRID_WIDTH columns.
grid = [
    [0, 0, 0, 0, 0, 1, 0, 0],
    [0, 1, 1, 0, 0, 1, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 1],
    [0, 0, 1, 1, 1, 0, 0, 0],
    [0, 0, 0, 0, 1, 0, 1, 0],
    [1, 0, 0, 0, 0, 0, 1, 0],
    [0, 0, 1, 1, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 1, 0, 0],
]

# Define a single, static goal. Change this tuple to whatever you want.
STATIC_GOAL = (7, 7)

# Define the starting cell, to which we reset after any violation.
START_CELL = (0, 0)

# Colors
WHITE = (255, 255, 255)
BLACK = (  0,   0,   0)
BLUE  = (  0,   0, 255)
RED   = (255,   0,   0)
GRAY  = (200, 200, 200)

# Window size
WINDOW_WIDTH  = GRID_WIDTH  * CELL_SIZE
WINDOW_HEIGHT = GRID_HEIGHT * CELL_SIZE

# Time (seconds) between each grid-step when moving
STEP_DELAY = 0.2  # seconds

# -------------------------------
# Helper functions for pygame
# -------------------------------
def cell_to_pixel_center(cell):
    """Convert an (col, row) grid cell to the pixel center (x, y)."""
    col, row = cell
    x = col * CELL_SIZE + CELL_SIZE // 2
    y = row * CELL_SIZE + CELL_SIZE // 2
    return (x, y)

def draw_grid(surface):
    """Draw the entire grid: white for free cells, black for obstacles."""
    for r in range(GRID_HEIGHT):
        for c in range(GRID_WIDTH):
            rect = pygame.Rect(c * CELL_SIZE, r * CELL_SIZE, CELL_SIZE, CELL_SIZE)
            color = BLACK if grid[r][c] == 1 else WHITE
            pygame.draw.rect(surface, color, rect)
            pygame.draw.rect(surface, GRAY, rect, 1)  # thin border

def draw_goal(surface, goal_cell):
    """Draw the goal as a red square inset by 4 pixels."""
    c, r = goal_cell
    rect = pygame.Rect(c * CELL_SIZE + 4, r * CELL_SIZE + 4,
                       CELL_SIZE - 8,    CELL_SIZE - 8)
    pygame.draw.rect(surface, RED, rect)

def draw_player(surface, player_cell):
    """Draw the player as a blue circle centered in the given cell."""
    x, y = cell_to_pixel_center(player_cell)
    radius = CELL_SIZE // 2 - 4
    pygame.draw.circle(surface, BLUE, (x, y), radius)

# -------------------------------
# The Action Server Node
# -------------------------------
class NavGameActionServer(Node):
    def __init__(self):
        super().__init__('nav_game')  # node name = 'nav_game'
        self._action_server = ActionServer(
            self,
            Waypoints,
            'waypoints',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # Initialize pygame (in the same thread as ROS!)
        pygame.init()
        self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption("nav_game: 2D Grid")
        self.clock = pygame.time.Clock()

        # Current cell starts at START_CELL.
        self.current_cell = START_CELL

        # The red goal square is ALWAYS drawn at STATIC_GOAL.
        self.goal_cell = STATIC_GOAL

        # Protect current_cell updates with a Lock
        self._lock = threading.Lock()

        # Create a 30 Hz timer to drive the pygame draw loop
        self._timer = self.create_timer(1.0 / 30.0, self._pygame_loop)

        self.get_logger().info("nav_game node started; waiting for Waypoints goals...")
        self.get_logger().info(f"Static goal is at {STATIC_GOAL}")

    def goal_callback(self, goal_request):
        self.get_logger().info('Received new goal request with %d waypoint(s).' %
                               len(goal_request.waypoints))
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request. Aborting...')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """
        Called when a new goal is accepted.
        The goal: goal_handle.request.waypoints (list of geometry_msgs/Point).
        Convert each Point → integer grid cell (col=int(x), row=int(y)).
        For each waypoint:
          • If NOT straight-aligned (row or column match), treat as violation: abort immediately.
          • Otherwise, step one grid‐cell at a time:
              – If obstacle encountered, abort and reset to START_CELL.
              – If reached, add to successful_waypoints and continue.
        Publish feedback (geometry_msgs/Point[] trajectory) at each step.
        After any violation, reset current_cell to START_CELL and return.
        """
        self.get_logger().info('Executing goal...')
        raw_waypoints = goal_handle.request.waypoints

        # Convert ROS Points → grid cells (col, row)
        waypoints = []
        for pt in raw_waypoints:
            col = int(pt.x)
            row = int(pt.y)
            waypoints.append((col, row))

        feedback_msg = Waypoints.Feedback()
        result = Waypoints.Result()
        successful_tuples = []  # store each reached waypoint as (col, row) tuples

        for wp in waypoints:
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled. Stopping execution.')
                goal_handle.canceled()
                # Reset to START_CELL on cancel as well:
                with self._lock:
                    self.current_cell = START_CELL
                return Waypoints.Result()

            # Check straight‐line alignment: same row OR same column
            with self._lock:
                cur_c, cur_r = self.current_cell
            target_c, target_r = wp

            if not (cur_c == target_c or cur_r == target_r):
                # Violation: not straight-aligned
                self.get_logger().error(
                    f"Waypoint {wp} is not straight-aligned from current {self.current_cell}! Aborting and resetting."
                )
                goal_handle.abort()

                # Prepare result
                result.successful_waypoints = []
                for (sc, sr) in successful_tuples:
                    p = Point()
                    p.x = float(sc)
                    p.y = float(sr)
                    p.z = 0.0
                    result.successful_waypoints.append(p)

                # Reset current_cell to START_CELL
                with self._lock:
                    self.current_cell = START_CELL

                return result

            # Build the list of intermediate cells from current → waypoint
            path_cells = []
            if cur_c == target_c:
                # vertical move
                step = 1 if target_r > cur_r else -1
                for rr in range(cur_r + step, target_r + step, step):
                    path_cells.append((cur_c, rr))
            else:
                # horizontal move
                step = 1 if target_c > cur_c else -1
                for cc in range(cur_c + step, target_c + step, step):
                    path_cells.append((cc, cur_r))

            # Traverse that straight‐line path one cell at a time
            for next_cell in path_cells:
                # Delay so movement is visible
                time.sleep(STEP_DELAY)

                # Publish feedback: add next_cell to feedback_msg.trajectory
                pt_msg = Point()
                pt_msg.x = float(next_cell[0])
                pt_msg.y = float(next_cell[1])
                pt_msg.z = 0.0
                feedback_msg.trajectory.append(pt_msg)
                goal_handle.publish_feedback(feedback_msg)

                # Check for obstacle in the grid
                col_idx, row_idx = next_cell
                if grid[row_idx][col_idx] == 1:
                    self.get_logger().error(f"Collision at {next_cell}! Aborting and resetting.")
                    goal_handle.abort()

                    # Prepare result
                    result.successful_waypoints = []
                    for (sc, sr) in successful_tuples:
                        p = Point()
                        p.x = float(sc)
                        p.y = float(sr)
                        p.z = 0.0
                        result.successful_waypoints.append(p)

                    # Reset current_cell to START_CELL
                    with self._lock:
                        self.current_cell = START_CELL

                    return result

                # No obstacle: update current_cell
                with self._lock:
                    self.current_cell = next_cell

                # **Immediately redraw so user sees the circle move step by step**
                self._redraw_now()

            # Reached this waypoint successfully
            successful_tuples.append(wp)
            # We do NOT update self.goal_cell here, since the red goal is static.

        # All waypoints processed without violation or collision → success
        self.get_logger().info('All waypoints processed. Succeeded.')

        # Convert successful_tuples → geometry_msgs/Point[] for the result
        result.successful_waypoints = []
        for (sc, sr) in successful_tuples:
            p = Point()
            p.x = float(sc)
            p.y = float(sr)
            p.z = 0.0
            result.successful_waypoints.append(p)

        # Note: on full success, we do NOT reset, so the player stays at the last waypoint.
        goal_handle.succeed()
        return result

    def _redraw_now(self):
        """
        Redraw the entire frame immediately, so that each move is visible in real time.
        """
        # Clear and draw background
        self.screen.fill(GRAY)
        draw_grid(self.screen)

        # Draw the static red goal
        draw_goal(self.screen, self.goal_cell)

        # Draw the blue player circle at its current position
        with self._lock:
            draw_player(self.screen, self.current_cell)

        pygame.display.flip()
        # We do NOT call clock.tick() here, because we only want one immediate frame

    def _pygame_loop(self):
        """
        Called at ~30 Hz by a ROS 2 timer.
        Processes Pygame events and redraws the grid, the static goal, and the player.
        """
        # Handle Pygame window events
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                self.get_logger().info("Pygame window closed; shutting down node.")
                pygame.quit()
                rclpy.shutdown()
                return

        # Draw background and player each tick
        self.screen.fill(GRAY)
        draw_grid(self.screen)
        draw_goal(self.screen, self.goal_cell)
        with self._lock:
            draw_player(self.screen, self.current_cell)

        pygame.display.flip()
        self.clock.tick(30)


def main(args=None):
    rclpy.init(args=args)
    node = NavGameActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # On shutdown: ensure pygame is quit
    node.get_logger().info("Shutting down nav_game node...")
    pygame.quit()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
