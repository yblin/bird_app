#ifndef DIALOG_MODIFY_NAME_H_
#define DIALOG_MODIFY_NAME_H_

#include <QDialog>

namespace Ui {
class DialogModifyName;
}

class DialogModifyName : public QDialog {
    Q_OBJECT

public:
    explicit DialogModifyName(QWidget *parent = nullptr);
    ~DialogModifyName();

    QString original_scientific_name() const;
    QString original_chinese_name() const;
    QString modified_scientific_name() const;

    void set_original_scientific_name(const std::string&);

private:
    Ui::DialogModifyName* ui_;
};

#endif // DIALOG_MODIFY_NAME_H_
