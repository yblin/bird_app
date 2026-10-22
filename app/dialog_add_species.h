#ifndef DIALOG_ADD_NEW_SPECIES_H_
#define DIALOG_ADD_NEW_SPECIES_H_

#include <QDialog>

namespace Ui {
class DialogAddSpecies;
}

class DialogAddSpecies : public QDialog {
    Q_OBJECT

public:
    explicit DialogAddSpecies(QWidget *parent = nullptr);
    ~DialogAddSpecies();

    QString scientific_name() const;
    QString chinese_name() const;
    QString english_name() const;
    void set_scientific_name(const QString& str);
    void set_chinese_name(const QString& str);
    void set_english_name(const QString& str);

private:
    Ui::DialogAddSpecies* ui_;
};

#endif // DIALOG_ADD_NEW_SPECIES_H_
