enum wallStakeStates {
  Idle,
  Loading,
  Scoring,
  Reset,
};

struct wallStake {
  wallStakeStates states;
  bool isCompleted;
};

extern wallStake wallstake;

void moveWallStake (double target, wallStake wallstake, double power);
int wallStakeTask();

