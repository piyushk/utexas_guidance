#include <class_loader/class_loader.h>
#include <utexas_guidance/mdp/visualizer.h>
#include <utexas_planning/common/exceptions.h>

namespace utexas_guidance {

  StateViewer::~StateViewer() {}

  void StateViewer::startEpisode(const utexas_planning::State::ConstPtr& start_state) {
    boost::mutex::scoped_lock lock(mutex_);
    current_state_ = boost::dynamic_pointer_cast<const State>(start_state);
    if (!current_state_) {
      throw utexas_planning::DowncastException("utexas_planning::State", "utexas_guidance::State");
    }
    next_state_.reset();
    time_at_state_update_ = boost::posix_time::microsec_clock::local_time();
    time_to_next_state_ = 0.0;
  }

  void StateViewer::updateState(const utexas_planning::State::ConstPtr& state_base, float timeout) {
    boost::mutex::scoped_lock lock(mutex_);
    State::ConstPtr state = boost::dynamic_pointer_cast<const State>(state_base);
    if (!state) {
      throw utexas_planning::DowncastException("utexas_planning::State", "utexas_guidance::State");
    }
    if (next_state_) {
      current_state_ = next_state_;
    }
    if (timeout == 0.0f) {
      current_state_ = state;
    } else {
      next_state_ = state;
    }
    time_to_next_state_ = timeout;
    time_at_state_update_ = boost::posix_time::microsec_clock::local_time();
  }

  void StateViewer::drawState(const State::ConstPtr& state) {


  }

  void StateViewer::drawInterpolatedState(const State::ConstPtr& state1,
                                          const State::ConstPtr& state2,
                                          float ratio) {


  }

  void StateViewer::init() {
    boost::mutex::scoped_lock lock(mutex_);
    glDisable(GL_LIGHTING);
    glPointSize(10.0f);
    glLineWidth(3.0f);
    setGridIsDrawn(false);
  }

  void StateViewer::draw() {
    boost::mutex::scoped_lock lock(mutex_);
    boost::posix_time::ptime current_time = boost::posix_time::microsec_clock::local_time();
    float time_since_state_update = (current_time - time_at_state_update_).total_milliseconds();
    if (!next_state_) {
      drawState(current_state_);
    } else if (time_since_state_update > time_to_next_state_) {
      current_state_ = next_state_;
      next_state_.reset();
      drawState(current_state_);
    } else {
      drawInterpolatedState(current_state_, next_state_, time_since_state_update / time_to_next_state_);
    }
  }

  Visualizer::~Visualizer() {}

  void Visualizer::init(int argc, char* argv[]) {
    QApplication application(argc, argv);
    viewer_.reset(new StateViewer);
    viewer_->setWindowTitle("GuidanceVisualizer");
    viewer_->show();
  }

  void Visualizer::startEpisode(const utexas_planning::State::ConstPtr& start_state) {
    viewer_->startEpisode(start_state);
  }

  void Visualizer::updateState(const utexas_planning::State::ConstPtr& state, float timeout) {
    viewer_->updateState(state, timeout);
  }

  void Visualizer::exec() {
    application_->exec();
  }

} /* utexas_planning */

CLASS_LOADER_REGISTER_CLASS(utexas_guidance::Visualizer, utexas_planning::Visualizer);
