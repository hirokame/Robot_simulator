# ============================================================
#  Robot Simulator — Step-by-Step Kata
#  Koji's Learning Session | 2026-03-15
# ============================================================

# --- STEP 1: Define the directions in clockwise order ---
# Index:  0    1    2    3
DIRECTIONS = ['N', 'E', 'S', 'W']

# --- STEP 2: Define how 'Advance' moves the robot ---
ADVANCE_MAP = {
    'N': (0,  1),   # North  → y goes up
    'S': (0, -1),   # South  → y goes down
    'E': (1,  0),   # East   → x goes right
    'W': (-1, 0),   # West   → x goes left
}

# ============================================================
#  The Robot class
# ============================================================
class Robot:
    def __init__(self, x, y, direction):
        """
        Create a robot at position (x, y) facing `direction`.
        direction must be one of: 'N', 'S', 'E', 'W'
        """
        self.x = x
        self.y = y
        self.direction = direction

    def turn_right(self):
        """Rotate 90° clockwise."""
        index = DIRECTIONS.index(self.direction)
        self.direction = DIRECTIONS[(index + 1) % 4]

    def turn_left(self):
        """Rotate 90° counter-clockwise."""
        index = DIRECTIONS.index(self.direction)
        self.direction = DIRECTIONS[(index - 1) % 4]

    def advance(self):
        """Move one step forward in the current direction."""
        dx, dy = ADVANCE_MAP[self.direction]
        self.x += dx
        self.y += dy

    def execute(self, commands):
        """
        Execute a string of commands.
        Each character must be 'L', 'R', or 'A'.
        """
        for cmd in commands:
            if cmd == 'R':
                self.turn_right()
            elif cmd == 'L':
                self.turn_left()
            elif cmd == 'A':
                self.advance()
            else:
                raise ValueError(f"Unknown command: '{cmd}'")

    def position(self):
        return (self.x, self.y)

    def __repr__(self):
        return f"Robot at ({self.x}, {self.y}) facing {self.direction}"


# ============================================================
#  STEP 3: Run test cases and show results
# ============================================================
def run_test(label, x, y, direction, commands, expected_pos, expected_dir):
    robot = Robot(x, y, direction)
    robot.execute(commands)
    pos_ok  = robot.position() == expected_pos
    dir_ok  = robot.direction  == expected_dir
    status  = "✅ PASS" if (pos_ok and dir_ok) else "❌ FAIL"
    print(f"{status}  [{label}]")
    print(f"       Start   : ({x}, {y}) facing {direction}")
    print(f"       Commands: {commands}")
    print(f"       Expected: {expected_pos} facing {expected_dir}")
    print(f"       Got     : {robot.position()} facing {robot.direction}")
    print()
    return pos_ok and dir_ok


if __name__ == "__main__":
    print("=" * 55)
    print("  🤖  Robot Simulator — Koji's Test Run  🤖")
    print("=" * 55)
    print()

    results = []

    # Test 1: Simple advance
    results.append(run_test(
        label="Advance North 3 times",
        x=0, y=0, direction='N',
        commands="AAA",
        expected_pos=(0, 3), expected_dir='N'
    ))

    # Test 2: Turn right and advance
    results.append(run_test(
        label="Turn right then advance",
        x=0, y=0, direction='N',
        commands="RAAAA",
        expected_pos=(4, 0), expected_dir='E'
    ))

    # Test 3: Turn left and advance
    results.append(run_test(
        label="Turn left then advance",
        x=0, y=0, direction='N',
        commands="LAAA",
        expected_pos=(-3, 0), expected_dir='W'
    ))

    # Test 4: Classic kata example
    results.append(run_test(
        label="Classic kata: RAALAL",
        x=0, y=0, direction='N',
        commands="RAALAL",
        expected_pos=(2, 1), expected_dir='W'
    ))

    # Test 5: Complex path
    # Trace: E →RR→ W, AAAAA(×5) moves West → (-3,-7), LL turns E, A → (-2,-7)
    results.append(run_test(
        label="Complex: start at (2, -7) facing E",
        x=2, y=-7, direction='E',
        commands="RRAAAAALLA",
        expected_pos=(-2, -7), expected_dir='E'
    ))

    # Test 6: Full 360° turn
    results.append(run_test(
        label="Full 360° right turn (no movement)",
        x=5, y=5, direction='S',
        commands="RRRR",
        expected_pos=(5, 5), expected_dir='S'
    ))

    # Summary
    passed = sum(results)
    total  = len(results)
    print("=" * 55)
    print(f"  Results: {passed}/{total} tests passed")
    print("=" * 55)
