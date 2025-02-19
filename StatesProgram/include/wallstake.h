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
wallStake wallstake;

void moveWallStake (double target, wallStake wallstake, double power);
int wallStakeTask();

