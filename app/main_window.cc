#include "dialog_add_label.h"
#include "dialog_add_species.h"
#include "dialog_modify_name.h"
#include "main_window.h"
#include "ui_main_window.h"

#include <QDesktopServices>
#include <QDirIterator>
#include <QFileDialog>
#include <QMessageBox>
#include <QModelIndex>
#include <QPainter>
#include <QScreen>
#include <QStandardItemModel>
#include <QTreeView>
#include <QTimer>
#include <QUrl>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <vector>

#include "codelibrary/base/array.h"
#include "codelibrary/string/string_split.h"
#include "codelibrary/string/string_trim.h"

namespace {

static QString GetIconFileName(TaxonomicTree::Item* item) {
    if (!item) return "";
    return "icons/" +
           QString::fromStdString(item->order->scientific_name) + "/" +
           QString::fromStdString(item->family->scientific_name) + "/" +
           QString::fromStdString(item->scientific_name) + ".jpg";
}

static QString GetDistributionFileName(TaxonomicTree::Item* item) {
    if (!item) return "";
    return "icons/" +
           QString::fromStdString(item->order->scientific_name) + "/" +
           QString::fromStdString(item->family->scientific_name) + "/" +
           QString::fromStdString(item->scientific_name) + "_distribution.png";
}

static QString GetIconFileName(TaxonomicTree::Item* item,
                               const QString& new_scientific_name) {
    if (!item) return "";
    std::string str = "icons/" + item->order->scientific_name + "/" +
                      item->family->scientific_name + "/" +
                      new_scientific_name.toStdString() + ".jpg";
    return QString::fromStdString(str);
}

static QString GetDistributionFileName(TaxonomicTree::Item* item,
                                       const QString& new_scientific_name) {
    if (!item) return "";
    std::string str = "icons/" + item->order->scientific_name + "/" +
                      item->family->scientific_name + "/" +
                      new_scientific_name.toStdString() + "_distribution.png";
    return QString::fromStdString(str);
}

static QString GetSoundPath(TaxonomicTree::Item* item) {
    if (!item) return "";
    std::string str = "sounds/" + item->order->scientific_name + "/" +
                      item->family->scientific_name + "/";
    return QString::fromStdString(str);
}

static std::string ConservationText(const ConservationState& state) {
    switch (state) {
    case UNKNOWN:
        return "";
    case LEAST_CONCERN:
        return "<font color=\"#68BB53\">无危</font>";
    case NEAR_THREATENED:
        return "<font color=\"#D1E744\">近危</font>";
    case VULNERABLE:
        return "<font color=\"#E5D42D\">易危</font>";
    case ENDANGERED:
        return "<font color=\"#E77B4D\">濒危</font>";
    case CRITICALLY_ENDANGERED:
        return "<font color=\"#B61917\">极危</font>";
    case EXTINCT_IN_THE_WILD:
        return "<span style=\"background-color:#542344;\">"
               "<font color=\"#FFFFFF\">野外灭绝</font></span>";
    case EXTINCT:
        return "<span style=\"background-color:#000000;\">"
               "<font color=\"#FFFFFF\">已灭绝</font></span>";
    case DATA_DEFICIENT:
        return "<font color=\"#595959\">无数据</font>";
    }

    return "";
}

// Map a 1.0-6.0 difficulty score to one of the 6 bands (recursive-median
// levels): 0=菜(<2.0), 1=易(2.0-3.0), 2=中(3.0-4.0), 3=难(4.0-5.0),
// 4=很难(5.0-6.0), 5=极难(>=6.0).
static int DifficultyBand(double score) {
    if (score < 2.0) return 0;
    if (score < 3.0) return 1;
    if (score < 4.0) return 2;
    if (score < 5.0) return 3;
    if (score < 6.0) return 4;
    return 5;
}

// Render the difficulty score as whole pentagram symbols. There is no clean
// half-star text glyph, so a half star is rounded to the nearest whole star
// for display (the exact score still shows the decimal).
static std::string StarString(double score) {
    int n = static_cast<int>(std::round(score));
    if (n < 1) n = 1;
    if (n > 6) n = 6;
    std::string s;
    for (int i = 0; i < n; ++i) s += "★";
    return s;
}

}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui_(new Ui::MainWindow) {
    ui_->setupUi(this);

    connect(ui_->taxonomic_tree, SIGNAL(pressed(QModelIndex)), this,
            SLOT(ShowItems(QModelIndex)));
    connect(ui_->taxonomic_tree, SIGNAL(activated(QModelIndex)), this,
            SLOT(ShowItems(QModelIndex)));

    // Show tools.
    connect(ui_->item_list, SIGNAL(activated(QModelIndex)), this,
            SLOT(ShowItem(QModelIndex)));
    connect(ui_->item_list, SIGNAL(pressed(QModelIndex)), this,
            SLOT(ShowItem(QModelIndex)));
    connect(ui_->item_list, SIGNAL(doubleClicked(QModelIndex)), this,
            SLOT(OpenImage(QModelIndex)));
    connect(ui_->check_box_observed, SIGNAL(clicked()), this,
            SLOT(UpdateObserved()));
    connect(ui_->check_box_only_listened, SIGNAL(clicked()), this,
            SLOT(UpdateOnlyListened()));
    connect(ui_->push_button_load_icon, SIGNAL(clicked(bool)), this,
            SLOT(LoadSpeciesImage()));
    connect(ui_->push_button_load_distribution, SIGNAL(clicked(bool)), this,
            SLOT(LoadDistribution()));
    connect(ui_->action_open_distribution, SIGNAL(triggered(bool)), this,
            SLOT(OpenDistributionImage()));
    connect(ui_->push_button_reload, SIGNAL(clicked(bool)), this,
            SLOT(Reload()));

    // Edit tools.
    ui_->push_button_save->setEnabled(false);
    connect(ui_->push_button_save, SIGNAL(clicked(bool)), this, SLOT(Save()));
    connect(ui_->text_edit_locations, SIGNAL(textChanged()), this,
            SLOT(UpdateLocations()));
    connect(ui_->text_edit_information, SIGNAL(textChanged()), this,
            SLOT(UpdateInformation()));
    connect(ui_->action_mark_all, SIGNAL(triggered(bool)), this,
            SLOT(MarkAll()));
    connect(ui_->action_change_name, SIGNAL(triggered(bool)), this,
            SLOT(ChangeName()));
    connect(ui_->action_add_species, SIGNAL(triggered(bool)), this,
            SLOT(AddSpecies()));
    connect(ui_->action_remove_species, SIGNAL(triggered(bool)), this,
            SLOT(RemoveSpecies()));
    connect(ui_->action_edit_taxonomic_tree, SIGNAL(triggered(bool)), this,
            SLOT(EditTaxonomicTree()));
    connect(ui_->action_add_endemic_label, SIGNAL(triggered(bool)), this,
            SLOT(SetEndemicLabel()));

    // Generation tools.
    connect(ui_->action_lifer_list, SIGNAL(triggered(bool)), this,
            SLOT(LiferList()));
    connect(ui_->action_observed_list, SIGNAL(triggered(bool)), this,
            SLOT(ObservedList()));
    connect(ui_->action_icon_list, SIGNAL(triggered(bool)), this,
            SLOT(IconList()));
    connect(ui_->push_button_export, SIGNAL(clicked(bool)), this,
            SLOT(IconList()));
    connect(ui_->action_export_to_html, SIGNAL(triggered(bool)), this,
            SLOT(Export()));
    connect(ui_->action_generate_label, SIGNAL(triggered(bool)), this,
            SLOT(GenerateLabel()));
    connect(ui_->action_export_sounds, SIGNAL(triggered(bool)), this,
            SLOT(ExportSounds()));

    // View mode.
    connect(ui_->action_view_mode, SIGNAL(triggered(bool)), this,
            SLOT(ViewMode()));
    connect(ui_->action_mark, SIGNAL(triggered(bool)), this,
            SLOT(SetMarked()));
    connect(ui_->action_mark_observed, SIGNAL(triggered(bool)), this,
            SLOT(SetObserved()));
    connect(ui_->action_show_non_rare_birds, SIGNAL(triggered(bool)), this,
            SLOT(ShowNonRareBirds()));
    connect(ui_->action_show_endemic_birds, SIGNAL(triggered(bool)), this,
            SLOT(ShowEndemicBirds()));
    connect(ui_->action_show_unobserved_birds, SIGNAL(triggered(bool)), this,
            SLOT(ShowUnobservedBirds()));
    connect(ui_->action_open_filter_file, SIGNAL(triggered(bool)), this,
            SLOT(OpenFilterFile()));
    connect(ui_->action_show_record_count, SIGNAL(triggered(bool)), this,
            SLOT(ShowRecordCount()));
    connect(ui_->action_show_difficulty_cai, SIGNAL(triggered(bool)), this,
            SLOT(ShowDifficultyCai()));
    connect(ui_->action_show_difficulty_yi, SIGNAL(triggered(bool)), this,
            SLOT(ShowDifficultyYi()));
    connect(ui_->action_show_difficulty_zhong, SIGNAL(triggered(bool)), this,
            SLOT(ShowDifficultyZhong()));
    connect(ui_->action_show_difficulty_nan, SIGNAL(triggered(bool)), this,
            SLOT(ShowDifficultyNan()));
    connect(ui_->action_show_difficulty_hennan, SIGNAL(triggered(bool)), this,
            SLOT(ShowDifficultyHenNan()));
    connect(ui_->action_show_difficulty_jinan, SIGNAL(triggered(bool)), this,
            SLOT(ShowDifficultyJiNan()));
    connect(ui_->action_select_all_difficulty, SIGNAL(triggered(bool)), this,
            SLOT(SelectAllDifficulty()));
    connect(ui_->action_cancel_all_difficulty, SIGNAL(triggered(bool)), this,
            SLOT(CancelAllDifficulty()));

    // Show conservation filter.
    connect(ui_->action_select_all_iucn, SIGNAL(triggered(bool)), this,
            SLOT(SelectAllIUCN()));
    connect(ui_->action_cancel_all_iucn, SIGNAL(triggered(bool)), this,
            SLOT(CancelAllIUCN()));
    connect(ui_->action_show_least_concern, SIGNAL(triggered(bool)), this,
            SLOT(ShowLeastConcernBirds()));
    connect(ui_->action_show_near_threatened, SIGNAL(triggered(bool)), this,
            SLOT(ShowNearThreatenedBirds()));
    connect(ui_->action_show_vulnerable, SIGNAL(triggered(bool)), this,
            SLOT(ShowVulnerableBirds()));
    connect(ui_->action_show_endangered, SIGNAL(triggered(bool)), this,
            SLOT(ShowEndangeredBirds()));
    connect(ui_->action_show_critically_endangered, SIGNAL(triggered(bool)),
            this,
            SLOT(ShowCriticallyEndangeredBirds()));
    connect(ui_->action_show_extinct_in_the_wild, SIGNAL(triggered(bool)), this,
            SLOT(ShowExtinctInTheWildBirds()));
    connect(ui_->action_show_extinct, SIGNAL(triggered(bool)), this,
            SLOT(ShowExtinctBirds()));
    connect(ui_->action_show_no_data, SIGNAL(triggered(bool)), this,
            SLOT(ShowNoDataBirds()));
    connect(ui_->action_show_others, SIGNAL(triggered(bool)), this,
            SLOT(ShowOtherBirds()));
    connect(ui_->action_load_iucn_list, SIGNAL(triggered(bool)), this,
            SLOT(LoadIUCNList()));

    // Sounds.
//    connect(ui_->push_button_load_sound, SIGNAL(clicked(bool)), this,
//            SLOT(LoadSound()));
    connect(ui_->sound_list, SIGNAL(pressed(QModelIndex)), this,
            SLOT(PlayAudio(QModelIndex)));

    // Other tools.
    connect(ui_->push_button_show_forward, SIGNAL(clicked(bool)), this,
            SLOT(ShowForward()));
    connect(ui_->push_button_show_backward, SIGNAL(clicked(bool)), this,
            SLOT(ShowBackward()));
    connect(ui_->push_button_search, SIGNAL(clicked(bool)), this,
            SLOT(Search()));
    connect(ui_->line_edit_search, SIGNAL(returnPressed()), this,
            SLOT(Search()));
    connect(ui_->line_edit_search, SIGNAL(textEdited(QString)), this,
            SLOT(SearchEdit(QString)));

    last_open_dir_ = "./";
    connect(ui_->action_filter_all, SIGNAL(triggered(bool)), this,
            SLOT(SetFilter()));
    ui_->action_filter_all->setData("All");
    current_filter_ = ui_->action_filter_all;

    QList<QPushButton*> tool_list;
    tool_list.push_back(ui_->push_button_save);
    tool_list.push_back(ui_->push_button_export);
    tool_list.push_back(ui_->push_button_reload);
    tool_list.push_back(ui_->push_button_load_icon);
    tool_list.push_back(ui_->push_button_load_distribution);
    tool_list.push_back(ui_->push_button_show_forward);
    tool_list.push_back(ui_->push_button_show_backward);

    ui_->sound_list->setModel(&sounds_model_);

    // Set item icon size.
    QScreen* screen = QGuiApplication::primaryScreen();
    QRect mm = screen->availableGeometry();
    int screen_width = mm.width();
    if (screen_width > 3000) {
        ui_->item_list->setIconSize(QSize(250, 250));
        for (QPushButton* button : tool_list) {
            button->setIconSize(QSize(64, 64));
        }
        ui_->push_button_search->setIconSize(QSize(32, 32));
        ui_->line_edit_search->setMinimumHeight(32);
    } else {
        for (QPushButton* button : tool_list) {
            button->setIconSize(QSize(32, 32));
        }
        ui_->push_button_search->setIconSize(QSize(24, 24));
        ui_->line_edit_search->setMinimumHeight(24);
        ui_->item_list->setIconSize(QSize(128, 128));
    }

    // Set accounts.
    account_menu_ = new QMenu(this);
    account_menu_->setTitle("账户");
    InitializeAccounts();

    // Set filters.
    recent_filter_menu_ = new QMenu(this);
    recent_filter_menu_->setTitle("Recents");
    ui_->menu_filter->clear();
    ui_->menu_filter->addAction(ui_->action_filter_all);
    ui_->menu_filter->addMenu(recent_filter_menu_);
    ui_->menu_filter->addMenu(account_menu_);
    ui_->menu_filter->addSeparator();
    filter_map_.clear();
    InitializeFilters("./filter", ui_->menu_filter);

    QTimer::singleShot(100, this, SLOT(Initialize()));

    timer_ = new QTimer(this);
    connect(timer_, SIGNAL(timeout()), this, SLOT(LoadIcons()));
}

MainWindow::~MainWindow() {
    delete ui_;
}

void MainWindow::Initialize() {
    LoadDifficultyData();
    ImportTaxonomicTree();

    // Add recently filters.
    QFile file("filters.txt");
    if (file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QTextStream fin(&file);

        QString str;
        recent_filter_.clear();
        while (fin.readLineInto(&str)) {
            QAction* action = new QAction(this);
            QFileInfo info(str);
            action->setText(info.baseName());
            QDir dir("./");
            action->setData(dir.relativeFilePath(str));
            connect(action, SIGNAL(triggered(bool)), this, SLOT(SetFilter()));
            recent_filter_.append(action);
        }

        UpdateRecentFilterMenu();
    }
}

void MainWindow::LoadDifficultyData() {
    record_count_map_.clear();
    difficulty_score_map_.clear();

    QFile file("bird_difficulty.csv");
    if (!file.open(QIODevice::ReadOnly)) return;
    QString content = QString::fromUtf8(file.readAll());
    QStringList lines = content.split('\n');

    // Frequency of each record count and total species with real data.
    // Values -1 (not found) are skipped: they have no difficulty.
    std::map<int, int> freq;
    int m = 0;
    for (const QString& line : lines) {
        QString l = line.trimmed();
        if (l.isEmpty()) continue;
        QStringList f = l.split(',');
        if (f.size() < 4) continue;
        if (f[0].trimmed() == "中文名") continue;  // Header.
        bool ok = false;
        int cnt = f[3].trimmed().toInt(&ok);
        if (!ok) continue;
        if (cnt < 0) continue;  // -1 (not found): no difficulty.
        record_count_map_[f[2].trimmed().toStdString()] = cnt;
        freq[cnt]++;
        ++m;
    }

    // Distinct counts ascending; the max count is the easiest (rank 1).
    std::vector<int> counts;
    for (const auto& kv : freq) counts.push_back(kv.first);
    std::sort(counts.begin(), counts.end());

    // score = -log2(1 - rank/m), clamped to [0,10]. A species' rank is
    // 1 + the number of species with a strictly larger count (the first
    // position of its tie group in count-descending order), so all species
    // sharing a count get the same score. This yields exactly k at the
    // k-th population median (rank = m(1 - 2^-k)).
    std::map<int, double> count_score;
    int rank = 1;
    for (auto it = counts.rbegin(); it != counts.rend(); ++it) {
        // rank_end = last position of this tie group (count-descending), so
        // the rarest species reaches exactly 10.0 while ties share a score.
        int rank_end = rank + freq[*it] - 1;
        double ratio = static_cast<double>(rank_end) / m;
        // Score is the star rating itself in [1.0, 6.0]: 1 + (-log2(1-ratio))
        // equals k+1 at the k-th median; cap at 6.0 for the top band.
        double score = (ratio >= 1.0)
                           ? 6.0
                           : std::min(6.0, 1.0 - std::log2(1.0 - ratio));
        count_score[*it] = score;
        rank += freq[*it];
    }
    for (const auto& kv : record_count_map_) {
        difficulty_score_map_[kv.first] = count_score[kv.second];
    }

    // Register difficulty bands in the taxonomic tree for filtering.
    for (const auto& kv : difficulty_score_map_) {
        taxonomic_tree_.set_difficulty_band(kv.first,
                                            DifficultyBand(kv.second));
    }
}

std::string MainWindow::DifficultySuffix(const TaxonomicTree::Item* item) const {
    if (!show_record_count_) return "";
    if (item->conservation_state == EXTINCT) return "";
    auto it = record_count_map_.find(item->scientific_name);
    if (it == record_count_map_.end()) return "";  // No data (-1): no difficulty.
    int cnt = it->second;
    auto s = difficulty_score_map_.find(item->scientific_name);
    double score = (s == difficulty_score_map_.end()) ? 0.0 : s->second;
    std::string suffix = StarString(score) + " " +
                         QString::number(score, 'f', 2).toStdString() +
                         "（" + std::to_string(cnt) + "）";

    // Star-band colors: 1★/2★ green, 3★ 易危, 4★ 近危, 5★ 濒危, 6★ 极危.
    // Extinct is excluded above.
    std::string color;
    switch (DifficultyBand(score)) {
    case 0: color = "#68BB53"; break;  // 1★ (无危绿)
    case 1: color = "#68BB53"; break;  // 2★ (无危绿)
    case 2: color = "#E5D42D"; break;  // 3★ (易危黄)
    case 3: color = "#D1E744"; break;  // 4★ (近危黄绿)
    case 4: color = "#E77B4D"; break;  // 5★ (濒危橙)
    case 5: color = "#B61917"; break;  // 6★ (极危红)
    }
    if (color.empty()) return suffix;
    return "<font color=\"" + color + "\">" + suffix + "</font>";
}

void MainWindow::ShowRecordCount() {
    show_record_count_ = ui_->action_show_record_count->isChecked();
    UpdateItemList();
    // Refresh the detail frame if a species is currently shown.
    std::string name = ui_->label_scientific_name->text().toStdString();
    if (!name.empty()) LoadItem(name);
}

void MainWindow::ShowDifficultyCai() {
    taxonomic_tree_.show_difficulty_band(
                0, ui_->action_show_difficulty_cai->isChecked());
    taxonomic_tree_.FixupFilter();
    this->SetTaxonomicTreeTitle();
}

void MainWindow::ShowDifficultyYi() {
    taxonomic_tree_.show_difficulty_band(
                1, ui_->action_show_difficulty_yi->isChecked());
    taxonomic_tree_.FixupFilter();
    this->SetTaxonomicTreeTitle();
}

void MainWindow::ShowDifficultyZhong() {
    taxonomic_tree_.show_difficulty_band(
                2, ui_->action_show_difficulty_zhong->isChecked());
    taxonomic_tree_.FixupFilter();
    this->SetTaxonomicTreeTitle();
}

void MainWindow::ShowDifficultyNan() {
    taxonomic_tree_.show_difficulty_band(
                3, ui_->action_show_difficulty_nan->isChecked());
    taxonomic_tree_.FixupFilter();
    this->SetTaxonomicTreeTitle();
}

void MainWindow::ShowDifficultyHenNan() {
    taxonomic_tree_.show_difficulty_band(
                4, ui_->action_show_difficulty_hennan->isChecked());
    taxonomic_tree_.FixupFilter();
    this->SetTaxonomicTreeTitle();
}

void MainWindow::ShowDifficultyJiNan() {
    taxonomic_tree_.show_difficulty_band(
                5, ui_->action_show_difficulty_jinan->isChecked());
    taxonomic_tree_.FixupFilter();
    this->SetTaxonomicTreeTitle();
}

void MainWindow::SelectAllDifficulty() {
    ui_->action_show_difficulty_cai->setChecked(true);
    ui_->action_show_difficulty_yi->setChecked(true);
    ui_->action_show_difficulty_zhong->setChecked(true);
    ui_->action_show_difficulty_nan->setChecked(true);
    ui_->action_show_difficulty_hennan->setChecked(true);
    ui_->action_show_difficulty_jinan->setChecked(true);
    for (int b = 0; b < 6; ++b) {
        taxonomic_tree_.show_difficulty_band(b, true);
    }
    taxonomic_tree_.FixupFilter();
    this->SetTaxonomicTreeTitle();
}

void MainWindow::CancelAllDifficulty() {
    ui_->action_show_difficulty_cai->setChecked(false);
    ui_->action_show_difficulty_yi->setChecked(false);
    ui_->action_show_difficulty_zhong->setChecked(false);
    ui_->action_show_difficulty_nan->setChecked(false);
    ui_->action_show_difficulty_hennan->setChecked(false);
    ui_->action_show_difficulty_jinan->setChecked(false);
    for (int b = 0; b < 6; ++b) {
        taxonomic_tree_.show_difficulty_band(b, false);
    }
    taxonomic_tree_.FixupFilter();
    this->SetTaxonomicTreeTitle();
}

void MainWindow::UpdateRecentFilterMenu() {
    recent_filter_menu_->clear();
    for (auto i = recent_filter_.rbegin(); i != recent_filter_.rend(); ++i) {
        recent_filter_menu_->addAction(*i);
    }
}

void MainWindow::InitializeAccounts() {
    QDir dir("./");
    QStringList filter;
    filter << "*.csv";
    QStringList files = dir.entryList(filter, QDir::Files, QDir::Time);

    for (const QString& file : files) {
        if (file == "record.csv" || file.startsWith("record-")) {
            QFileInfo info(file);

            QAction* action;
            if (file == "record.csv") {
                action = new QAction("默认");
            } else {
                assert(file.size() >= 11);
                action = new QAction(file.mid(7, file.size() - 11));
            }
            action->setData(file);
            action->setCheckable(true);
            account_menu_->addAction(action);
            connect(action, SIGNAL(triggered(bool)), this, SLOT(SetAccount()));
            if (file == "record.csv") action->triggered(true);
        }
    }
}

void MainWindow::InitializeFilters(const QString& dir_path, QMenu* menu) {
    QDir dir(dir_path);
    QStringList files = dir.entryList(QDir::Files | QDir::AllDirs |
                                      QDir::NoDotAndDotDot,
                                      QDir::Name);
    for (const QString& file : files) {
        QFileInfo info(dir_path + "/" + file);
        if (info.isDir()) {
            QMenu* sub_menu = new QMenu(info.baseName());
            menu->addMenu(sub_menu);
            InitializeFilters(info.absoluteFilePath(), sub_menu);
        } else {
            QAction* action = new QAction(info.baseName());
            QDir dir("./");
            QString path = dir.relativeFilePath(info.filePath());
            action->setData(path);
            action->setCheckable(true);
            menu->addAction(action);
            QString name = info.fileName();
            name = name.left(name.indexOf('.'));
            filter_map_[name] = action;
            connect(action, SIGNAL(triggered(bool)), this, SLOT(SetFilter()));
        }
    }
}

void MainWindow::ImportTaxonomicTree() {
    QString error;
    error = taxonomic_tree_.LoadTree();
    if (!error.isEmpty()) {
        QMessageBox::warning(this, "读取taxonomic_tree.csv错误", error);
        return;
    }
    error = taxonomic_tree_.LoadItems(current_account_);
    if (!error.isEmpty()) {
        QMessageBox::warning(this, "载入文件错误", error);
        return;
    }
    taxonomic_tree_.CountItems();
    SetTaxonomicTreeTitle();
}

void MainWindow::SetAccount() {
    for (QAction* action : account_menu_->actions()) {
        action->setChecked(false);
    }
    QAction* action = dynamic_cast<QAction*>(sender());
    action->setChecked(true);
    if (modified_) this->Save();
    current_account_ = action->data().toString();
    this->Reload();
}

void MainWindow::SetFilter() {
    QAction* action = dynamic_cast<QAction*>(sender());
    this->SetFilter(action);
}

void MainWindow::SetFilter(QAction* action) {
    if (current_filter_) {
        current_filter_->setChecked(false);
    }
    action->setChecked(true);
    current_filter_ = action;

    for (int i = 0; i < recent_filter_.size(); ++i) {
        QFileInfo info1(recent_filter_.at(i)->data().toString());
        QFileInfo info2(action->data().toString());

        if (info1.baseName() == info2.baseName()) {
            recent_filter_.removeAt(i);
            break;
        }
    }
    if (recent_filter_.size() > 10) recent_filter_.removeFirst();
    if (action->data() != "All")
        recent_filter_.append(action);

    this->UpdateRecentFilterMenu();
    this->ResetFilter();
}

void MainWindow::ResetFilter() {
    this->taxonomic_tree_.LoadFilter(current_filter_->data().toString().
                                     toLocal8Bit().toStdString());
    SetTaxonomicTreeTitle();
}

void MainWindow::Reload() {
    if (modified_) {
        QMessageBox::StandardButton rb =
                QMessageBox::question(this, "birding", "已修改，是否保存?",
                                      QMessageBox::Yes | QMessageBox::No,
                                      QMessageBox::No);
        if (rb == QMessageBox::Yes) {
            this->Save();
        }
    }

    this->ImportTaxonomicTree();
    this->ResetFilter();
}

void MainWindow::HideSearchResultLabel() {
    ui_->label_search_result->setText("");
}

/**
 * Export to HTML.
 */
void MainWindow::Export() {
    int n_total_items = 0;
    for (TaxonomicTree::Order* order : taxonomic_tree_.orders()) {
        n_total_items += order->n_items;
    }

    std::string html;
    html = "<html>\n"
           "<body>\n"
           "<style>\n"
           "@media print { html, body { height: 99%; } }\n"
           "</style>\n";

    std::string table = "<table border=0 style=\"font-size:10px; "
                        "background-color:#FFFFFF\">\n";
    const int n_items_per_page = 3;
    int n_items = 0;

    for (TaxonomicTree::Order* order : taxonomic_tree_.orders()) {
        if (order->n_items == 0) continue;
        int n_count = 0;

        html += "<h2>" + order->chinese_name + "</h2>\n";
        html += "<hr/>\n";

        for (TaxonomicTree::Family* family : order->famliers) {
            if (family->n_items == 0) continue;

            html += "<h3>" + family->chinese_name + "</h3>\n";

            cl::Array<TaxonomicTree::Item*> items;
            taxonomic_tree_.GetItems(family->scientific_name, &items);

            for (const auto& item : items) {
                QString icon_path = GetIconFileName(item);
                QString dist_path = GetDistributionFileName(item);
                std::string dist_img =
                        "<img src=\"" + dist_path.toStdString() + "\" border=1 "
                        "height=113px align=right "
                        "onerror=\'this.src=\"no_pic.jpg\"; this.border=0\'"
                        ">";
                std::string img = "<img src=\"" + icon_path.toStdString() +
                                  "\" height=250px>";

                html += table;
                html += "  <td width=280px>" + img + "</td>\n";
                html += "  <td width=420px valign=top>\n";

                // Distribution and names.
                html += "    <br/><br/>\n";
                html += "    <p>" + dist_img + "</p>\n";
                html += "    <p>" + item->chinese_name;
                html += "&nbsp;&nbsp;&nbsp;&nbsp;";

                html += ConservationText(item->conservation_state);
                html += "</p>\n";

                html += "    <p>" +
                        item->english_name + "</p>\n";
                html += "    <p>" +
                        item->scientific_name + "</p>\n";

                // Information.
                html += "    <p>";
                bool first_line = true;
                for (const auto& s : item->information) {
                    if (!s.empty()) {
                        if (first_line) {
                            html += s;
                            first_line = false;
                        } else {
                            QString str = QString::fromStdString(s);
                            if (str.startsWith("⚑")) {
                                html += "<li>" + s.substr(3) + "</li>";
                            }
                        }
                    }
                }
                html += "    </p>\n";
                html += "  </td>\n";

                html += "</table>\n";
                html += "<br />\n";
                ++n_items;
                if (++n_count % n_items_per_page == 0 &&
                    n_items != n_total_items) {
                    html += "<div style=\"page-break-after: always;\">"
                            "</div>\n";
                }
            }
        } // family
        if (n_count % n_items_per_page != 0 && n_items != n_total_items) {
            html += "<div style=\"page-break-after: always;\"></div>\n";
        }
    }

    html += "<hr/>\n";
    html += "<p>Generated by L(yblin@Xiamen)</p>";
    html += "</body>\n"
            "</html>\n";

    std::ofstream of("out.html");
    of << html;
    of.close();
    system("out.html");
}

/**
 * Export current sounds to local path.
 */
void MainWindow::ExportSounds() {
    int count = 0;
    QString save_dir = QDir::currentPath() + "/export_sounds";
    QDir target_dir(save_dir);
    if (!target_dir.exists()) {
        target_dir.mkpath(".");
    } else {
        QFileInfoList file_list =
                target_dir.entryInfoList(QDir::Files | QDir::NoDotAndDotDot);
        for (const QFileInfo& file_info : file_list) {
            QFile file(file_info.absoluteFilePath());
            file.remove();
        }
    }

    for (TaxonomicTree::Order* order : taxonomic_tree_.orders()) {
        if (order->n_items == 0) continue;

        for (TaxonomicTree::Family* family : order->famliers) {
            if (family->n_items == 0) continue;

            cl::Array<TaxonomicTree::Item*> items;
            taxonomic_tree_.GetItems(family->scientific_name, &items);

            for (TaxonomicTree::Item* item : items) {
                QString sound_path = GetSoundPath(item);
                QFileInfo src_info(sound_path);
                if (!src_info.exists() || !src_info.isDir()) continue;

                QString sci_name =
                        QString::fromStdString(item->scientific_name);
                QString match_prefix = sci_name;
                QString cn_name = QString::fromStdString(item->chinese_name);

                QDir src_dir(sound_path);
                QString filter = QString("%1*.mp3").arg(match_prefix);
                src_dir.setNameFilters({filter});
                src_dir.setFilter(QDir::Files | QDir::NoDotAndDotDot);
                QFileInfoList sub_files = src_dir.entryInfoList();

                for (const QFileInfo& file_info : sub_files) {
                    QString origin_name = file_info.fileName();
                    std::string cn_name = item->chinese_name;
                    QString qcn_name = QString::fromStdString(cn_name);

                    QString new_file_name = QString("%1_%2").arg(qcn_name,
                                                                 origin_name);
                    QString target_path = target_dir.filePath(new_file_name);
                    QFile src_file(file_info.absoluteFilePath());
                    bool copy_ok = src_file.copy(target_path);
                    if (!copy_ok) {
                        LOG(ERROR) << "Failed copy dir file: "
                                   << file_info.absoluteFilePath().toStdString()
                                   << " target:"
                                   << target_path.toStdString()
                                   << " Err:"
                                   << src_file.errorString().toStdString();
                    } else {
                        ++count;
                    }
                }
            }
        } // family
    }

    QString msg = QString("音频导出完成！\n成功导出文件总数：%1 个\n文件保存目录：%2")
                  .arg(count).arg(save_dir);
    QMessageBox::information(this, "导出完成", msg);
}

void MainWindow::ShowForward() {
    if (current_show_index_ + 1 < show_history_.size()) {
        LocateItem(show_history_[++current_show_index_]);
    }
}

void MainWindow::ShowBackward() {
    if (current_show_index_ - 1 >= 0) {
        LocateItem(show_history_[--current_show_index_]);
    }
}

void MainWindow::LocateItem(const std::string& name) {
    QStandardItem* item = taxonomic_tree_.Search(name);
    if (!item) return;

    ui_->taxonomic_tree->setCurrentIndex(item->index());
    this->ShowItems(item->index());

    QStandardItemModel* model = static_cast<QStandardItemModel*>(
                ui_->item_list->model());
    if (!model) return ;

    const std::string scientific_name = taxonomic_tree_.GetItemName(name);
    LoadItem(scientific_name);
    UpdateForwardAndBackward();
}

bool MainWindow::Search(const QString& str) {
    if (str.at(0) == '-') {
        if (filter_map_.find(str.mid(1)) == filter_map_.end()) return false;

        this->SetFilter(filter_map_[str.mid(1)]);
        return true;
    }

    QStandardItem* item = taxonomic_tree_.Search(str.toStdString());
    if (!item) {
        return false;
    }

    ui_->taxonomic_tree->setCurrentIndex(item->index());
    this->ShowItems(item->index());

    QStandardItemModel* model = static_cast<QStandardItemModel*>(
                ui_->item_list->model());
    if (!model) return true;

    const std::string scientific_name =
            taxonomic_tree_.GetItemName(str.toStdString());
    for (int i = 0; i < model->rowCount(); ++i) {
        QStandardItem* item = model->item(i);
        std::string name = item->text().toStdString();

        cl::Array<std::string> terms;
        cl::StringSplit(name, '\n', &terms);
        if (terms.empty()) continue;

        const std::string t = taxonomic_tree_.GetItemName(terms[1]);
        if (!t.empty() && t == scientific_name) {
            ShowItem(item->index());
            return true;
        }
    }
    return true;
}

void MainWindow::Search() {
    QCompleter* completer = ui_->line_edit_search->completer();
    if (!completer) return;

    QString completed_str = completer->currentCompletion();
    QString str = ui_->line_edit_search->text().trimmed();

    if (!Search(str)) {
        if (completed_str.contains(str, Qt::CaseInsensitive) &&
            Search(completed_str))
            ui_->line_edit_search->setText(completed_str);
        else {
            ui_->label_search_result->
                    setText("<Font color=\"red\">未找到</Font>");
            QTimer::singleShot(1000, this, SLOT(HideSearchResultLabel()));
        }
    }
}

void MainWindow::SearchEdit(const QString& str) {
    if (str.isEmpty()) return;

    QChar c = str.at(0);
    if (!c.isUpper() && !c.isLower()) {
        ui_->line_edit_search->setCompleter(chinese_search_completer_);
    } else {
        if (str.length() >= 3)
            ui_->line_edit_search->setCompleter(search_completer_);
        else
            ui_->line_edit_search->setCompleter(nullptr);
    }
}

void MainWindow::ViewMode() {
    ++view_mode_;
    if (view_mode_ == 4) view_mode_ = 0;

    switch(view_mode_) {
    case 0:
        ui_->frame_information->show();
        break;
    case 1:
        ui_->taxonomic_tree->hide();
        break;
    case 2:
        ui_->frame_information->hide();
        break;
    case 3:
        ui_->taxonomic_tree->show();
        break;
    }

    QTimer::singleShot(0, this, SLOT(UpdateItemList()));
}

void MainWindow::OpenFilterFile() {
    LOG(ERROR) << "???";
    QString path = current_filter_->data().toString();
    if (path == "All") {
        QDesktopServices::openUrl(QUrl::fromLocalFile("birds.csv"));
    } else {
        QDesktopServices::openUrl(QUrl::fromLocalFile(path));
    }
    LOG(ERROR) << "???";
}

void MainWindow::UpdateItemList() {
    ShowItems(ui_->taxonomic_tree->currentIndex());
}

void MainWindow::LoadSpeciesImage() {
    std::string name = ui_->label_scientific_name->text().toStdString();
    if (name.empty()) return;

    TaxonomicTree::Item* item = taxonomic_tree_.get_item(name);
    if (!item) return;

    QString str = QFileDialog::getOpenFileName(
                this,
                QString("Load Image"),
                last_open_dir_,
                QString("Image files(*.jpg *.png *.jfif)"));
    if (!str.isEmpty()) {
        QImage image;
        image.load(str);

        QString icon_file = GetIconFileName(item);
        QString save_path = icon_file;
        QFileInfo file_info(save_path);
        QDir dir;

        if (!dir.exists(file_info.absolutePath())) {
            if (!dir.mkpath(file_info.absolutePath())) {
                assert(false && "Cannot create path.");
            }
        }

        int size = qMax(image.width(), image.height());

        QImage square_image(size, size, image.format());
        square_image.fill(Qt::white);

        QPainter painter(&square_image);
        int xoffset = (size - image.width()) / 2;
        int yoffset = size - image.height();

        painter.drawImage(QPoint(xoffset, yoffset), image);
        painter.end();

        square_image.save(save_path, "JPG", 100);

        last_open_dir_ = QFileInfo(str).filePath();
        this->UpdateIcon();
        if (item->genus->item) {
            this->ShowItems(item->genus->item->index());
        }
    }
}

void MainWindow::LoadDistribution() {
    std::string name = ui_->label_scientific_name->text().toStdString();
    if (name.empty()) return;

    TaxonomicTree::Item* item = taxonomic_tree_.get_item(name);
    if (!item) return;

    QString str = QFileDialog::getOpenFileName(
                this,
                QString("Load distribution"),
                last_open_dir_,
                QString("JPG files(*.jpg *.png *.gif);;"
                        "Image files(*.*)"));

    if (!str.isEmpty()) {
        QFileInfo file_info(str);

        if (file_info.suffix().toLower() == "png") {
            QString target = GetDistributionFileName(item);
            if (QFile::exists(target)) {
                QFile::remove(target);
            }
            QFile::copy(str, target);
        } else {
            QImage image;
            image.load(str);

            QImage png_image(image.width(), image.height(),
                             QImage::Format_ARGB32);
            QPainter painter(&png_image);
            painter.drawImage(QPoint(0, 0), image);
            painter.end();

            QString file = GetDistributionFileName(item);
            png_image.save(file, "PNG");
        }

        last_open_dir_ = file_info.filePath();
        this->UpdateDistribution();
    }
}

void MainWindow::SetTaxonomicTreeTitle() {
    QStandardItemModel* model = static_cast<QStandardItemModel*>(
                ui_->taxonomic_tree->model());
    if (model) model->clear();
    ui_->taxonomic_tree->setModel(taxonomic_tree_.SetupModel());

    int n_observed = 0;
    int n_all = 0;
    for (TaxonomicTree::Order* order : taxonomic_tree_.orders()) {
        n_observed += order->n_observed_items;
        n_all += order->n_items;
    }

    QString account;
    if (current_account_ == "record.csv") {
        account = "默认";
    } else {
        assert(current_account_.size() >= 11);
        account = current_account_.mid(7, current_account_.size() - 11);
    }

    this->setWindowTitle(QString("已观测:") +
                         QString::number(n_observed) + "-" +
                         QString::number(n_all) +
                         "(" +
                         current_filter_->text() +
                         ")" + " - " + account);

    QStringList chinese_word_list, word_list;
    for (TaxonomicTree::Order* order : taxonomic_tree_.orders()) {
        if (order->n_items == 0) continue;

        for (TaxonomicTree::Family* family : order->famliers) {
            if (family->n_items == 0) continue;
            chinese_word_list << QString::fromStdString(family->chinese_name);
            word_list << QString::fromStdString(family->english_name);
            word_list << QString::fromStdString(family->scientific_name);

            for (TaxonomicTree::Genus* genus : family->genus) {
                if (genus->n_items == 0) continue;
                chinese_word_list <<
                        QString::fromStdString(genus->chinese_name);
                word_list << QString::fromStdString(genus->english_name);
                word_list << QString::fromStdString(genus->scientific_name);

                genus->item->setText(
                            genus->item->text() + " [" +
                            QString::number(genus->n_observed_items) + "-" +
                            QString::number(genus->n_items) + "]");
                if (genus->n_observed_items == genus->n_items &&
                    genus->n_items != 0) {
                    genus->item->setBackground(
                                QBrush(QColor(0, 255, 0, 128)));
                }

                for (const auto& k : genus->items) {
                    auto item = k.second;
                    if (!taxonomic_tree_.InFilter(item->scientific_name))
                        continue;

                    chinese_word_list <<
                            QString::fromStdString(item->chinese_name);
                    word_list << QString::fromStdString(item->english_name);
                    word_list << QString::fromStdString(item->scientific_name);
                }
            }
            family->item->setText(
                        family->item->text() + " [" +
                        QString::number(family->n_observed_items) + "-" +
                        QString::number(family->n_items) + "]");
            if (family->n_observed_items == family->n_items &&
                family->n_items != 0) {
                family->item->setBackground(QBrush(QColor(0, 255, 0, 128)));
            }
        }

        if (order->n_observed_items == order->n_items &&
            order->n_items != 0) {
            order->item->setBackground(QBrush(QColor(0, 255, 0, 128)));
        }
    }

    for (auto p : filter_map_) {
        chinese_word_list << "-" + p.first;
    }

    chinese_search_completer_ = new QCompleter(chinese_word_list);
    chinese_search_completer_->setFilterMode(Qt::MatchContains);

    search_completer_ = new QCompleter(word_list);
    search_completer_->setFilterMode(Qt::MatchContains);
    search_completer_->setCaseSensitivity(Qt::CaseInsensitive);

//    if (ui_->line_edit_search->text().isEmpty())
//        ui_->line_edit_search->setCompleter(nullptr);
//    else {
//        QString text = ui_->line_edit_search->text();
//        if (text.at(0).isLower() || text.at(0).isUpper()) {
//            if (text.length() >= 3)
//                ui_->line_edit_search->setCompleter(search_completer_);
//            else
//                ui_->line_edit_search->setCompleter(nullptr);
//        } else {
//            ui_->line_edit_search->setCompleter(chinese_search_completer_);
//        }
//    }
}

void MainWindow::Save() {
    taxonomic_tree_.SaveItems(current_account_.toLocal8Bit().toStdString());
    modified_ = false;
    ui_->push_button_save->setEnabled(false);
}

void MainWindow::closeEvent(QCloseEvent*) {
    if (modified_) {
        QMessageBox::StandardButton rb =
                QMessageBox::question(this, "birding", "已修改，是否保存?",
                                      QMessageBox::Yes | QMessageBox::No,
                                      QMessageBox::No);
        if (rb == QMessageBox::Yes) {
            this->Save();
        }
    }

    QFile file("filters.txt");
    if (file.open(QFile::WriteOnly | QFile::Truncate)) {
        QTextStream out(&file);
        for (auto i : recent_filter_) {
            out << i->data().toString() << Qt::endl;
        }
    }
}

void MainWindow::UpdateLocations() {
    std::string name = ui_->label_scientific_name->text().toStdString();
    if (name.empty()) return;

    TaxonomicTree::Item* item = taxonomic_tree_.get_item(name);
    std::string locations;
    for (const std::string& t : item->observation_locations) {
        locations += t + ";";
    }

    std::string text = ui_->text_edit_locations->toPlainText().toStdString();
    std::replace(text.begin(), text.end(), '\n', ';');

    if (locations != text) {
        modified_ = true;
        cl::StringSplit(text, ';', &item->observation_locations);
        ui_->push_button_save->setEnabled(true);
    }
}

void MainWindow::UpdateInformation() {
    std::string name = ui_->label_scientific_name->text().toStdString();
    if (name.empty()) return;

    TaxonomicTree::Item* item = taxonomic_tree_.get_item(name);
    std::string information;
    for (const std::string& t : item->information) {
        information += t + ";";
    }

    std::string text = ui_->text_edit_information->toPlainText().toStdString();
    std::replace(text.begin(), text.end(), '\n', ';');

    if (information != text) {
        AfterModify();
        cl::StringSplit(text, ';', &item->information);
    }
}

void MainWindow::UpdateIcon() {
    std::string name = ui_->label_scientific_name->text().toStdString();
    if (name.empty()) return;

    TaxonomicTree::Item* item = taxonomic_tree_.get_item(name);
    QString dir = GetIconFileName(item);
    QPixmap image(dir);
    if (image.isNull()) {
        ui_->label_image->clear();
        return;
    }

    int w = ui_->label_image->width();
    int h = ui_->label_image->height();
    image = image.scaled(w, h, Qt::KeepAspectRatio, Qt::SmoothTransformation);
    ui_->label_image->setPixmap(image);
}

void MainWindow::UpdateDistribution() {
    std::string name = ui_->label_scientific_name->text().toStdString();
    if (name.empty()) return;

    TaxonomicTree::Item* item = taxonomic_tree_.get_item(name);
    QString dir = GetDistributionFileName(item);
    QPixmap image(dir);
    if (image.isNull()) {
        ui_->label_distribution->clear();
        return;
    }

    int w = ui_->label_distribution->width();
    int h = ui_->label_distribution->height();
    image = image.scaled(w, h, Qt::KeepAspectRatio, Qt::SmoothTransformation);
    ui_->label_distribution->setPixmap(image);
}

void MainWindow::UpdateSounds() {
    sounds_model_.clear();

    std::string name = ui_->label_scientific_name->text().toStdString();
    if (name.empty()) return;

    TaxonomicTree::Item* item = taxonomic_tree_.get_item(name);
    if (!item) return;

    QIcon icon(":/icon/resources/load_sound.png");

    QDir dir(GetSoundPath(item));
    QStringList file_list = dir.entryList();

    foreach (QString file, file_list) {
        if (file.startsWith(QString::fromStdString(name))) {
            QString name = file.section('_', 1);
            if (name.isEmpty()) name = "叫声";
            else {
                int index = name.indexOf('.');
                if (index != -1) {
                    name = name.left(index);
                }
            }
            QStandardItem* item = new QStandardItem(icon, name);
            item->setData(file, Qt::UserRole);
            sounds_model_.appendRow(item);
        }
    }
}

bool MainWindow::LoadItem(const std::string& name) {
    TaxonomicTree::Item* item = taxonomic_tree_.GetItem(name);
    if (!item) return false;

    std::string conservation = ConservationText(item->conservation_state);
    std::string state;
    if (item->is_rare()) {
        state = "<font color=\"light green\">稀见</font>";
    }

    if (item->is_endemic()) {
        std::string endemic_str;
        if (item->endemic_information.empty()) {
            endemic_str = "<font color=\"brown\">特有种</font>";
        } else {
            endemic_str = "<font color=\"brown\">" + item->endemic_information +
                          "</font>";
        }
        state = state.empty() ? endemic_str : state + " " + endemic_str;
    }

    std::string chinese_name_plus_info_str =
            item->chinese_name + "  " + conservation + "  " + state;
    std::string difficulty_suffix = DifficultySuffix(item);
    if (!difficulty_suffix.empty()) {
        chinese_name_plus_info_str += "  " + difficulty_suffix;
    }
    QString chinese_name_plus_info =
            QString::fromStdString(chinese_name_plus_info_str);
    ui_->label_chinese_name->setText(chinese_name_plus_info);
    ui_->label_english_name->setText(QString::fromStdString(
                                         item->english_name));
    ui_->label_scientific_name->setText(QString::fromStdString(
                                            item->scientific_name));
    ui_->check_box_observed->setChecked(item->is_observed());
    ui_->check_box_only_listened->setChecked(item->is_only_listened());

    QString locations;
    for (const std::string& location : item->observation_locations) {
        locations += QString::fromStdString(location) + "\n";
    }
    ui_->text_edit_locations->setPlainText(locations);

    QString information;
    for (const std::string& t : item->information) {
        information += QString::fromStdString(t) + "\n";
    }
    ui_->text_edit_information->setPlainText(information);
    UpdateIcon();
    UpdateDistribution();
    UpdateSounds();

    return true;
}

void MainWindow::UpdateForwardAndBackward() {
    if (current_show_index_ + 1 < show_history_.size()) {
        this->ui_->push_button_show_forward->setEnabled(true);
    } else {
        this->ui_->push_button_show_forward->setEnabled(false);
    }
    if (current_show_index_ > 0) {
        this->ui_->push_button_show_backward->setEnabled(true);
    } else {
        this->ui_->push_button_show_backward->setEnabled(false);
    }
}

void MainWindow::ShowItem(const QModelIndex& index) {
    if (!ui_->item_list->model()) return;

    QStandardItemModel* model = static_cast<QStandardItemModel*>(
                ui_->item_list->model());
    QString str = model->itemData(index).values()[0].toString();
    std::string name = str.toStdString();

    cl::Array<std::string> terms;
    cl::StringSplit(name, '\n', &terms);
    if (LoadItem(terms[1])) {
        // Update history.
        const int n_lasts = 10;
        int prev = std::max(show_history_.size() - n_lasts, 0);
        bool found = false;
        for (int i = show_history_.size() - 1; i >= prev; --i) {
            if (show_history_[i] == terms[1]) {
                for (int j = i + 1; j < show_history_.size(); ++j) {
                    show_history_[j - 1] = show_history_[j];
                }
                show_history_.back() = terms[1];
                current_show_index_ = show_history_.size() - 1;
                found = true;
            }
        }

        if (!found) {
            while (current_show_index_ + 1 < show_history_.size()) {
                show_history_.pop_back();
            }
            if (show_history_.empty() || terms[1] != show_history_.back()) {
                ++current_show_index_;
                show_history_.push_back(terms[1]);
            }
        }
        UpdateForwardAndBackward();
    }
    return;
}

void MainWindow::UpdateObserved() {
    std::string name = ui_->label_scientific_name->text().toStdString();
    if (name.empty()) return;

    TaxonomicTree::Item* item = taxonomic_tree_.get_item(name);
    assert(item);

    if (!ui_->check_box_observed->isChecked()) {
        if (item->is_observed()) {
            AfterModify();
            item->set_unobserved();
        }
    } else {
        if (!item->is_observed()) {
            AfterModify();
            item->set_observed();
        }
    }
    UpdateItemList();
}

void MainWindow::UpdateOnlyListened() {
    std::string name = ui_->label_scientific_name->text().toStdString();
    if (name.empty()) return;

    TaxonomicTree::Item* item = taxonomic_tree_.get_item(name);
    if (!ui_->check_box_only_listened->isChecked()) {
        if (item->is_only_listened()) {
            AfterModify();
            item->set_unlistened();
        }
    } else {
        if (!item->is_only_listened()) {
            AfterModify();
            item->set_listened();
        }
    }
}

void MainWindow::SetMarked() {
    std::string name = ui_->label_scientific_name->text().toStdString();
    if (name.empty()) return;

    TaxonomicTree::Item* item = taxonomic_tree_.get_item(name);
    if (item->is_marked()) {
        item->set_unmarked();
    } else {
        item->set_marked();
    }
    AfterModify();
    UpdateItemList();
}

void MainWindow::SetObserved() {
    std::string name = ui_->label_scientific_name->text().toStdString();
    if (name.empty()) return;

    TaxonomicTree::Item* item = taxonomic_tree_.get_item(name);

    if (item->is_observed()) {
        item->set_unobserved();
    } else {
        item->set_observed();
    }
    AfterModify();
    ui_->check_box_observed->setChecked(item->is_observed());
    UpdateItemList();
}

void MainWindow::MarkAll() {
    for (TaxonomicTree::Order* order : taxonomic_tree_.orders()) {
        if (order->n_items == 0) continue;

        for (TaxonomicTree::Family* family : order->famliers) {
            if (family->n_items == 0) continue;

            for (TaxonomicTree::Genus* genus : family->genus) {
                if (genus->n_items == 0) continue;

                for (const auto& k : genus->items) {
                    auto item = k.second;
                    if (!taxonomic_tree_.InFilter(item->scientific_name))
                        continue;

                    item->set_observed();
                }
            }
        }
    }
    AfterModify();
}

void MainWindow::LiferList() {
    taxonomic_tree_.SaveLiferList();
    QDesktopServices::openUrl(QUrl::fromLocalFile("./lifer_list.csv"));
}

void MainWindow::ObservedList() {
    taxonomic_tree_.SaveObservedList();
    QDesktopServices::openUrl(QUrl::fromLocalFile("./observed_list.csv"));
}

void MainWindow::AfterModify() {
    modified_ = true;
    ui_->push_button_save->setEnabled(true);
}

/**
 * Generate a HTML file including to illustrate all icons.
 */
void MainWindow::IconList() {
    int n_total_items = 0;
    for (TaxonomicTree::Order* order : taxonomic_tree_.orders()) {
        n_total_items += order->n_items;
    }

    std::string html;
    html = "<html>\n"
           "<body>\n";

    std::string table = "<table border=0 style=\"font-size:10px; "
                        "background-color:#FFFFFF\">\n";
    std::string break_page = "<div style=\"page-break-after: always;\">"
                             "</div>\n";

    const int n_items_per_line = 4;
    const int n_lines_per_page = 4;
    const int n_families_per_page = 3;
    int n = 0;
    int n_lines = 0;
    int n_families = 0;

    for (TaxonomicTree::Order* order : taxonomic_tree_.orders()) {
        if (order->n_items == 0) continue;

        for (TaxonomicTree::Family* family : order->famliers) {
            if (family->n_items == 0) continue;
            int n_items = 0;

            if (n_lines + 1 == n_lines_per_page &&
                n_families + 1 == n_families_per_page) {
                html += break_page;
                n_lines = 0;
                n_families = 0;
            }
            html += "<h4 align=\"center\">" + family->chinese_name + "</h4>\n";
            html += "<hr/>\n";
            ++n_families;

            cl::Array<TaxonomicTree::Item*> items;
            taxonomic_tree_.GetItems(family->scientific_name, &items);

            for (const auto& item : items) {
                QString icon_path = GetIconFileName(item);
                std::string img = "<img src=\"" + icon_path.toStdString() +
                                  "\" height=200px>";

                if (n_items % n_items_per_line == 0) {
                    html += table;
                }
                html += "  <td width=256px align=\"center\">" + img + "\n";
                html += "  <p>" + item->chinese_name;
                html += "&nbsp;&nbsp;&nbsp;&nbsp;";
                html += ConservationText(item->conservation_state);
                if (item->is_endemic()) {
                    html += "&nbsp;&nbsp;&nbsp;&nbsp;";
                    html += "<font color=\"brown\">" +
                            item->endemic_information + "</font>";
                } else if (item->is_rare()) {
                    html += "&nbsp;&nbsp;&nbsp;&nbsp;";
                    html += "<font color=\"light green\">稀见</font>";
                }
                html += "</p>\n";
                html += "  <p>" + item->english_name + "</p>\n";
                html += "  </td>\n";

                if (++n_items % n_items_per_line == 0) {
                    html += "</table>\n";
                    ++n_lines;
                }
                if (++n == n_total_items) break;

                if (n_lines == n_lines_per_page ||
                    (n_lines == n_lines_per_page - 1 &&
                     n_families == n_families_per_page)) {
                    html += "<div style=\"page-break-after: always;\">"
                            "</div>\n";
                    n_lines = 0;
                    n_families = 0;
                }
            }
            if (n_items % n_items_per_line != 0) {
                html += "</table>\n";
                ++n_lines;
            }
            if (n == n_total_items) break;

            if (n_lines == n_lines_per_page ||
                (n_lines == n_lines_per_page - 1 &&
                 n_families == n_families_per_page)) {
                html += "<div style=\"page-break-after: always;\">"
                        "</div>\n";
                n_lines = 0;
                n_families = 0;
            }
        } // family
    }

    html += "<hr/>\n";
    html += "<p>Generated by L(yblin@Xiamen)</p>";
    html += "</body>\n"
            "</html>\n";

    std::ofstream of("out.html");
    of << html;
    of.close();
    system("out.html");
}

void MainWindow::GenerateLabel() {
    std::string name = ui_->label_scientific_name->text().toStdString();
    if (name.empty()) return;

    TaxonomicTree::Item* item = taxonomic_tree_.get_item(name);
    assert(item);

    QString info = QString::fromStdString(item->chinese_name) + "\n" +
                   QString::fromStdString(item->order->chinese_name) + "." +
                   QString::fromStdString(item->family->chinese_name) + "\n" +
                   QString::fromStdString(item->scientific_name) + "\n";
    for (auto s : item->observation_locations) {
        info += QString::fromStdString(s) + "\n";
    }

    QMessageBox msg;
    msg.setTextInteractionFlags(Qt::TextSelectableByMouse);
    msg.setText(info);
    msg.exec();
}

void MainWindow::ShowNonRareBirds() {
    taxonomic_tree_.set_only_non_rare_birds(
                ui_->action_show_non_rare_birds->isChecked());
    taxonomic_tree_.FixupFilter();
    this->SetTaxonomicTreeTitle();
}

void MainWindow::ShowEndemicBirds() {
    taxonomic_tree_.set_only_endemic_birds(
                ui_->action_show_endemic_birds->isChecked());
    taxonomic_tree_.FixupFilter();
    this->SetTaxonomicTreeTitle();
}

void MainWindow::ShowUnobservedBirds() {
    taxonomic_tree_.set_only_unobserved_birds(
                ui_->action_show_unobserved_birds->isChecked());
    taxonomic_tree_.FixupFilter();
    this->SetTaxonomicTreeTitle();
}

void MainWindow::SelectAllIUCN() {
    ui_->action_show_least_concern->setChecked(true);
    ui_->action_show_near_threatened->setChecked(true);
    ui_->action_show_vulnerable->setChecked(true);
    ui_->action_show_endangered->setChecked(true);
    ui_->action_show_critically_endangered->setChecked(true);
    ui_->action_show_extinct_in_the_wild->setChecked(true);
    ui_->action_show_extinct->setChecked(true);
    ui_->action_show_no_data->setChecked(true);
    ui_->action_show_others->setChecked(true);

    for (int i = 0; i < CONSERVATION_STRS.size(); ++i) {
        taxonomic_tree_.show_conservation_state(ConservationState(i), true);
    }
    taxonomic_tree_.FixupFilter();
    this->SetTaxonomicTreeTitle();
}

void MainWindow::CancelAllIUCN() {
    ui_->action_show_least_concern->setChecked(false);
    ui_->action_show_near_threatened->setChecked(false);
    ui_->action_show_vulnerable->setChecked(false);
    ui_->action_show_endangered->setChecked(false);
    ui_->action_show_critically_endangered->setChecked(false);
    ui_->action_show_extinct_in_the_wild->setChecked(false);
    ui_->action_show_extinct->setChecked(false);
    ui_->action_show_no_data->setChecked(false);
    ui_->action_show_others->setChecked(false);

    for (int i = 0; i < CONSERVATION_STRS.size(); ++i) {
        taxonomic_tree_.show_conservation_state(ConservationState(i), false);
    }
    taxonomic_tree_.FixupFilter();
    this->SetTaxonomicTreeTitle();
}

void MainWindow::ShowLeastConcernBirds() {
    taxonomic_tree_.show_conservation_state(
                LEAST_CONCERN,
                ui_->action_show_least_concern->isChecked());
    taxonomic_tree_.FixupFilter();
    this->SetTaxonomicTreeTitle();
}

void MainWindow::ShowNearThreatenedBirds() {
    taxonomic_tree_.show_conservation_state(
                NEAR_THREATENED,
                ui_->action_show_near_threatened->isChecked());
    taxonomic_tree_.FixupFilter();
    this->SetTaxonomicTreeTitle();
}

void MainWindow::ShowVulnerableBirds() {
    taxonomic_tree_.show_conservation_state(
                VULNERABLE,
                ui_->action_show_vulnerable->isChecked());
    taxonomic_tree_.FixupFilter();
    this->SetTaxonomicTreeTitle();
}

void MainWindow::ShowEndangeredBirds() {
    taxonomic_tree_.show_conservation_state(
                ENDANGERED,
                ui_->action_show_endangered->isChecked());
    taxonomic_tree_.FixupFilter();
    this->SetTaxonomicTreeTitle();
}

void MainWindow::ShowCriticallyEndangeredBirds() {
    taxonomic_tree_.show_conservation_state(
                CRITICALLY_ENDANGERED,
                ui_->action_show_critically_endangered->isChecked());
    taxonomic_tree_.FixupFilter();
    this->SetTaxonomicTreeTitle();
}

void MainWindow::ShowExtinctInTheWildBirds() {
    taxonomic_tree_.show_conservation_state(
                EXTINCT_IN_THE_WILD,
                ui_->action_show_extinct_in_the_wild->isChecked());
    taxonomic_tree_.FixupFilter();
    this->SetTaxonomicTreeTitle();
}

void MainWindow::ShowExtinctBirds() {
    taxonomic_tree_.show_conservation_state(
                EXTINCT,
                ui_->action_show_extinct->isChecked());
    taxonomic_tree_.FixupFilter();
    this->SetTaxonomicTreeTitle();
}

void MainWindow::ShowNoDataBirds() {
    taxonomic_tree_.show_conservation_state(
                DATA_DEFICIENT,
                ui_->action_show_no_data->isChecked());
    taxonomic_tree_.FixupFilter();
    this->SetTaxonomicTreeTitle();
}

void MainWindow::ShowOtherBirds() {
    taxonomic_tree_.show_conservation_state(
                UNKNOWN,
                ui_->action_show_others->isChecked());
    taxonomic_tree_.FixupFilter();
    this->SetTaxonomicTreeTitle();
}

void MainWindow::LoadIUCNList() {
    QString file = QFileDialog::getOpenFileName(this, QString("Load IUCN list"),
                                               last_open_dir_,
                                               QString("(*.csv)"));
    if (!file.isEmpty()) {
        cl::io::LineReader line_reader(file.toStdString());
        while (char* line = line_reader.ReadLine()) {
            if (strlen(line) == 0) continue;

            cl::Array<std::string> terms;
            cl::StringSplit(line, ',', &terms);
            assert(terms.size() >= 15);

            TaxonomicTree::Item* item = taxonomic_tree_.get_item(terms[2]);
            if (item) {
                ConservationState state = UNKNOWN;
                for (auto term : terms) {
                    for (int i = 0; i < CONSERVATION_STRS.size(); ++i) {
                        if (term == CONSERVATION_STRS[i] && term != "Unknown") {
                            state = static_cast<ConservationState>(i);
                            break;
                        }
                    }
                }

                if (item->conservation_state != state) {
                    LOG(INFO) << item->chinese_name << ": "
                              << CONSERVATION_STRS[item->conservation_state]
                              << "->"
                              << CONSERVATION_STRS[state];

                    item->conservation_state = state;
                }
            }
        }

        taxonomic_tree_.SaveIUCN();
    }
}

void MainWindow::AddSpecies() {
    this->Reload();

    DialogAddSpecies dialog;
    dialog.setWindowTitle("添加物种");
    if (dialog.exec() != QDialog::Accepted) return;

    if (dialog.chinese_name().trimmed().isEmpty()) {
        QMessageBox::warning(this, "错误", "中文名不能为空");
    } else if (dialog.english_name().trimmed().isEmpty()) {
        QMessageBox::warning(this, "错误", "英文名不能为空");
    } else if (dialog.scientific_name().trimmed().isEmpty()) {
        QMessageBox::warning(this, "错误", "学名不能为空");
    }

    TaxonomicTree::Item item;
    item.chinese_name = dialog.chinese_name().trimmed().toStdString();
    item.english_name = dialog.english_name().trimmed().toStdString();
    item.scientific_name = dialog.scientific_name().trimmed().toStdString();

    taxonomic_tree_.AddSpecies(item);
    this->Save();
    this->Reload();
}

void MainWindow::RemoveSpecies() {
    std::string name = ui_->label_scientific_name->text().toStdString();
    if (name.empty()) return;

    TaxonomicTree::Item* item = taxonomic_tree_.get_item(name);
    assert(item);

    taxonomic_tree_.RemoveSpecies(*item);

    this->UpdateItemList();
    AfterModify();
}

void MainWindow::SetEndemicLabel() {
    DialogAddLabel dialog;
    dialog.setWindowTitle("修改特种信息");
    std::string name = ui_->label_scientific_name->text().toStdString();
    if (name.empty()) return;

    TaxonomicTree::Item* item = taxonomic_tree_.get_item(name);
    if (!item) return;

    dialog.set_label(QString::fromStdString(item->endemic_information));

    int r = dialog.exec();

    if (r == QDialog::Accepted) {
        QString str = dialog.label().trimmed();
        if (str.isEmpty()) {
            item->state &= ~TaxonomicTree::ENDEMIC;
            item->endemic_information = "";
        } else {
            item->state |= TaxonomicTree::ENDEMIC;
            item->endemic_information = str.toStdString();
        }
        this->Save();
        this->Reload();
    }
}

void MainWindow::EditTaxonomicTree() {
    system("taxonomic_tree.csv");
    this->Reload();
}

void MainWindow::ChangeName() {
    DialogAddSpecies dialog;
    dialog.setWindowTitle("修改物种名");
    std::string name = ui_->label_scientific_name->text().toStdString();
    if (name.empty()) return;

    TaxonomicTree::Item* item = taxonomic_tree_.get_item(name);
    if (!item) return;

    dialog.set_scientific_name(QString::fromStdString(item->scientific_name));
    dialog.set_chinese_name(QString::fromStdString(item->chinese_name));
    dialog.set_english_name(QString::fromStdString(item->english_name));

    int r = dialog.exec();

    if (r == QDialog::Accepted) {
        QString str = dialog.scientific_name().trimmed();
        bool modify = false;
        modify = item->chinese_name != dialog.chinese_name().trimmed().
                                       toStdString();
        modify |= item->english_name != dialog.english_name().trimmed().
                                        toStdString();

        item->chinese_name = dialog.chinese_name().trimmed().toStdString();
        item->english_name = dialog.english_name().trimmed().toStdString();

        QString str1 = QString::fromStdString(item->scientific_name);
        if (str != str1) {
            QString file1_name = GetIconFileName(item);
            QString file2_name = GetDistributionFileName(item);
            QFile file1(file1_name);
            QFile file2(file2_name);

            QDir dir;
            QFileInfo info(file1_name);
            if (!dir.exists(info.absolutePath())) {
                if (!dir.mkpath(info.absolutePath())) {
                    QMessageBox::warning(this, "错误", "无法新建目录");
                    return;
                }
            }

            file1.rename(GetIconFileName(item, str));
            file2.rename(GetDistributionFileName(item, str));

            // Replace sounds.
            QDir dir1(GetSoundPath(item));
            if (!dir1.exists()) {
                if (!dir1.mkpath(".")) {
                    QMessageBox::warning(this, "错误", "无法新建目录");
                    return;
                }
            }
            QStringList file_list = dir.entryList();
            foreach (QString file, file_list) {
                if (file.startsWith(str1)) {
                    QFile file3(dir.absolutePath() + "//" + file);
                    file.replace(str1, str);
                    file3.rename(dir.absolutePath() + "//" + file);
                }
            }

            taxonomic_tree_.ModifyScientificName(str1.toStdString(),
                                                 str.toStdString());
            this->Save();
            this->Reload();
        } else if (modify) {
            UpdateItemList();
            AfterModify();
            this->Save();
            this->Reload();
        }
    }
}

QStandardItem* MainWindow::CreateListItem(TaxonomicTree::Item* item) const {
    QString title = QString::fromStdString(item->chinese_name + "\n" +
                                           item->english_name);
    QStandardItem* q_item = new QStandardItem(title);

    if (item->is_marked()) {
        q_item->setForeground(QBrush(QColor(0, 0, 128)));
    } else if (item->is_rare()) {
        q_item->setForeground(QBrush(QColor(0, 128, 0)));
    } else if (item->conservation_state == EXTINCT) {
        q_item->setForeground(QBrush(QColor(128, 128, 128)));
    } else if (item->is_endemic()) {
        q_item->setForeground(QBrush(QColor(128, 0, 0)));
    } else if (item->conservation_state == CRITICALLY_ENDANGERED ||
               item->conservation_state == EXTINCT_IN_THE_WILD) {
        q_item->setForeground(QBrush(QColor(255, 0, 0)));
    } else if (item->conservation_state == ENDANGERED) {
        q_item->setForeground(QBrush(QColor(231, 123, 77)));
    }

    int size = ui_->item_list->iconSize().width();
    if (size == 250) {
        q_item->setSizeHint(QSize(size + 90, size + 100));
    } else {
        q_item->setSizeHint(QSize(size + 30, size + 70));
    }
    return q_item;
}

void MainWindow::OpenImage(const QModelIndex& index) {
    QStandardItemModel* model = static_cast<QStandardItemModel*>(
                ui_->item_list->model());
    QString str = model->itemData(index).values()[0].toString();
    std::string name = str.toStdString();

    cl::Array<std::string> terms;
    cl::StringSplit(name, '\n', &terms);
    TaxonomicTree::Item* item = taxonomic_tree_.GetItem(terms[1]);
    assert(item);

    QString path = GetIconFileName(item);
    QDesktopServices::openUrl(QUrl::fromLocalFile(path));
}

void MainWindow::OpenDistributionImage() {
    std::string name = ui_->label_scientific_name->text().toStdString();
    if (name.empty()) return;

    TaxonomicTree::Item* item = taxonomic_tree_.get_item(name);
    if (!item) return;

    QString path = GetDistributionFileName(item);
    QDesktopServices::openUrl(QUrl::fromLocalFile(path));
}

void MainWindow::ShowItems(const QModelIndex& index) {
    QStandardItemModel* model = static_cast<QStandardItemModel*>(
                ui_->taxonomic_tree->model());
    if (!model || !ui_->item_list) return;

    if (model->itemData(index).values().empty()) return;
    QString str = model->itemData(index).values()[0].toString();
    std::string name = str.toStdString();

    std::string::size_type l = name.find('('), r = name.find(')');
    name = name.substr(l + 1, (r - l) - 1);

    cl::Array<TaxonomicTree::Item*> items;
    taxonomic_tree_.GetItems(name, &items);
    if (items.empty()) return ;

    QStandardItemModel* model1 = new QStandardItemModel(ui_->item_list);

    bool all_observed = true;
    bool all_non_observed = true;
    for (TaxonomicTree::Item* item : items) {
        if (!item->is_observed()) {
            all_observed = false;
            continue;
        } else {
            all_non_observed = false;
        }

        model1->appendRow(CreateListItem(item));
    }

    if (!all_observed && !all_non_observed) {
        // Add split items.
        QStandardItem* item0 = new QStandardItem();
        item0->setEnabled(false);
        item0->setSizeHint(QSize(ui_->item_list->width() - 4, 20));
        model1->appendRow(item0);

        QStandardItem* item1 = new QStandardItem();
        item1->setEnabled(false);
        item1->setSizeHint(QSize(ui_->item_list->width() - 4, 5));
        item1->setBackground(QBrush(QColor(225, 225, 225)));
        model1->appendRow(item1);

        QStandardItem* item2 = new QStandardItem();
        item2->setEnabled(false);
        item2->setSizeHint(QSize(ui_->item_list->width() - 4, 20));
        model1->appendRow(item2);
    }

    for (TaxonomicTree::Item* item : items) {
        if (item->is_observed()) continue;
        model1->appendRow(CreateListItem(item));
    }

    // Need release the memory.
    QStandardItemModel* model2 = static_cast<QStandardItemModel*>(
                ui_->item_list->model());
    if (model2) {
        model2->clear();
        delete model2;
    }

    ui_->item_list->setModel(model1);

    timer_->stop();

    // Start a new thread to load icons.
    timer_->start(10);
    current_icon_index_ = 0;
}

std::string MainWindow::GetItemScientificName(QStandardItem* item) const {
    if (!item) return "";

    std::string name = item->text().toStdString();
    cl::Array<std::string> terms;
    cl::StringSplit(name, '\n', &terms);
    if (terms.size() != 2) return "";

    return taxonomic_tree_.GetItemName(terms[1]);
}

void MainWindow::LoadIcons() {
    QStandardItemModel* model = static_cast<QStandardItemModel*>(
                ui_->item_list->model());
    if (!model) return;

    if (current_icon_index_ >= model->rowCount()) {
        timer_->stop();
        return;
    }

    QStandardItem* item = model->item(current_icon_index_++);
    CHECK(item);

    std::string name = GetItemScientificName(item);
    if (name.empty()) return;
    QString icon_file = GetIconFileName(taxonomic_tree_.get_item(name));
    QImage image(icon_file);
    if (image.isNull()) return;
    image = image.scaled(ui_->item_list->iconSize().width(),
                         ui_->item_list->iconSize().height(),
                         Qt::KeepAspectRatio,
                         Qt::SmoothTransformation);
    QPixmap pixelmap = QPixmap::fromImage(image);

    QIcon icon(pixelmap);
    item->setIcon(icon);
}

void MainWindow::LoadSound() {
    std::string name = ui_->label_scientific_name->text().toStdString();
    if (name.empty()) return;

    TaxonomicTree::Item* item = taxonomic_tree_.get_item(name);
    if (!item) return;

    QString str = QFileDialog::getOpenFileName(this, QString("Load sound"),
                                               last_open_dir_,
                                               QString("MP3 files(*.mp3);;"
                                                       "Sound files(*.*)"));
//    if (!str.isEmpty()) {
//        QString sound_file = GetSoundFileName(item);
//        QDir dir;
//        dir.remove(sound_file);
//        QFile::copy(str, sound_file);
//        last_open_dir_ = QFileInfo(str).filePath();
//        this->UpdateIcon();
//    }
}

void MainWindow::PlayAudio(const QModelIndex& index) {
    if (sounds_model_.itemData(index).values().size() != 3) return;
    QString str = sounds_model_.itemData(index).values()[2].toString();

    QString s = str.left(str.indexOf('.'));
    s = s.left(s.indexOf('_'));
    TaxonomicTree::Item* item = taxonomic_tree_.get_item(s.toStdString());

    if (item) {
        QString sound = GetSoundPath(item) + str;
        QDesktopServices::openUrl(QUrl::fromLocalFile(sound));
    }
}
