# ifndef RVIZ_EDITOR_PLUGINS_CSV_MARKER_DISPLAY_HPP
# define RVIZ_EDITOR_PLUGINS_CSV_MARKER_DISPLAY_HPP

# ifndef Q_MOC_RUN
# include <rclcpp/rclcpp.hpp>
# include <rviz_common/panel.hpp>
# include <QtWidgets>
# endif

namespace rviz_editor_plugins
{
class CsvMarkerDisplay : public rviz_common::Panel
{
  Q_OBJECT
public:
  CsvMarkerDisplay(QWidget * parent = nullptr);

  virtual void onInitialize();
  virtual void load(const rviz_common::Config & config);
  virtual void save(rviz_common::Config config) const;

public Q_SLOTS:
  void tick();

protected:
  rclcpp::Node::SharedPtr node_;
  QPushButton * load_button_;
  QPushButton * save_button_;
  void loadCsv();
  void saveCsv();
};

}

# endif // RVIZ_EDITOR_PLUGINS_CSV_MARKER_DISPLAY_HPP
  