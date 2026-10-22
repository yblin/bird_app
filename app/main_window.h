#ifndef MAIN_WINDOW_H_
#define MAIN_WINDOW_H_

#include <QCompleter>
#include <QList>
#include <QMainWindow>
#include <QModelIndex>
#include <QtNetwork/QNetworkAccessManager>

#include "taxonomic_tree.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private Q_SLOTS:
    void Initialize();
    void LoadSpeciesImage();
    void LoadIcons();
    void LoadDistribution();
    void LoadSound();
    void OpenImage(const QModelIndex& index);
    void OpenDistributionImage();
    void Save();
    void ShowItems(const QModelIndex& index);
    void ShowItem(const QModelIndex& index);

    void UpdateObserved();
    void UpdateOnlyListened();
    void UpdateLocations();
    void UpdateInformation();
    void UpdateItemList();
    void UpdateIcon();
    void UpdateDistribution();
    void UpdateSounds();

    void Reload();
    void SetAccount();
    void SetFilter();
    void SetFilter(QAction* action);
    void ResetFilter();
    void SetMarked();
    void MarkAll();
    void SetObserved();
    void AddSpecies();
    void RemoveSpecies();
    void EditTaxonomicTree();
    void SetEndemicLabel();
    void ChangeName();
    void AfterModify();

    void Search();
    bool Search(const QString&);
    void SearchEdit(const QString&);
    void HideSearchResultLabel();

    void LiferList();
    void ObservedList();
    void IconList();
    void GenerateLabel();
    void Export();
    void ExportSounds();

    void SelectAllIUCN();
    void CancelAllIUCN();
    void LoadIUCNList();
    void ShowNonRareBirds();
    void ShowEndemicBirds();
    void ShowUnobservedBirds();
    void ShowLeastConcernBirds();
    void ShowNearThreatenedBirds();
    void ShowVulnerableBirds();
    void ShowEndangeredBirds();
    void ShowCriticallyEndangeredBirds();
    void ShowExtinctInTheWildBirds();
    void ShowExtinctBirds();
    void ShowNoDataBirds();
    void ShowOtherBirds();
    void ViewMode();
    void OpenFilterFile();
    void ShowRecordCount();
    void ShowDifficultyCai();
    void ShowDifficultyYi();
    void ShowDifficultyZhong();
    void ShowDifficultyNan();
    void ShowDifficultyHenNan();
    void ShowDifficultyJiNan();
    void SelectAllDifficulty();
    void CancelAllDifficulty();

    void ShowForward();
    void ShowBackward();

    void PlayAudio(const QModelIndex& index);

private:
    void closeEvent(QCloseEvent* event) override;
    void ImportTaxonomicTree();
    void SetTaxonomicTreeTitle();
    void InitializeAccounts();
    void InitializeFilters(const QString& dir_path, QMenu* menu);
    bool LoadItem(const std::string& name);
    void LocateItem(const std::string& name);
    void UpdateForwardAndBackward();
    void UpdateRecentFilters();
    QStandardItem* CreateListItem(TaxonomicTree::Item* item) const;
    std::string GetItemScientificName(QStandardItem* item) const;
    void UpdateRecentFilterMenu();
    void LoadDifficultyData();
    std::string DifficultySuffix(const TaxonomicTree::Item* item) const;

    int view_mode_ = 0;
    bool modified_ = false;
    Ui::MainWindow* ui_;
    QString last_open_dir_;
    TaxonomicTree taxonomic_tree_;
    QString current_account_;
    QAction* current_filter_ = nullptr;
    int current_show_index_ = -1;
    cl::Array<std::string> show_history_;
    std::map<std::string, QByteArray> icon_map_;
    QTimer* timer_;
    int current_icon_index_ = 0;
    QMenu* recent_filter_menu_ = nullptr;
    QMenu* account_menu_ = nullptr;
    QList<QAction*> recent_filter_;
    QStandardItemModel sounds_model_;
    std::map<QString, QAction*> filter_map_;
    QCompleter* chinese_search_completer_ = nullptr;
    QCompleter* search_completer_ = nullptr;
    bool show_record_count_ = false;
    std::map<std::string, int> record_count_map_;
    std::map<std::string, double> difficulty_score_map_;
};

#endif // MAIN_WINDOW_H_
