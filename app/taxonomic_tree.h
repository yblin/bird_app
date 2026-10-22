#ifndef TAXONOMIC_TREE_H_
#define TAXONOMIC_TREE_H_

#include <fstream>
#include <map>
#include <set>
#include <string>

#include <QStandardItem>
#include <QStandardItemModel>

#include "codelibrary/base/array.h"
#include "codelibrary/base/pool.h"
#include "codelibrary/string/string_split.h"
#include "codelibrary/string/string_trim.h"
#include "codelibrary/util/io/line_reader.h"

// State of IUCN red list.
enum ConservationState {
    UNKNOWN  = 0,
    LEAST_CONCERN = 1,
    NEAR_THREATENED = 2,
    VULNERABLE = 3,
    ENDANGERED = 4,
    CRITICALLY_ENDANGERED = 5,
    EXTINCT_IN_THE_WILD = 6,
    EXTINCT = 7,
    DATA_DEFICIENT = 8
};

static cl::Array<std::string> CONSERVATION_STRS = {
    "Unknown",
    "Least Concern",
    "Near Threatened",
    "Vulnerable",
    "Endangered",
    "Critically Endangered",
    "Extinct in the Wild",
    "Extinct",
    "Data Deficient"
};

class TaxonomicTree {
public:
    enum ItemState {
        NONE          = 0,
        OBSERVED      = 1,
        MARKED        = 2,
        RARE          = 4,
        ENDEMIC       = 8,
        ONLY_LISTENED = 16
    };

    struct Order;
    struct Family;
    struct Genus;

    struct Item {
        Order* order;
        Family* family;
        Genus* genus;

        // Show or hide this item.
        bool show;

        int state = NONE;
        ConservationState conservation_state = UNKNOWN;
        std::string chinese_name;
        std::string scientific_name;
        std::string english_name;
        cl::Array<std::string> information;
        cl::Array<std::string> observation_locations;
        std::string endemic_information;

        bool is_observed()      const { return (state & OBSERVED) != 0;      }
        bool is_marked()        const { return (state & MARKED) != 0;        }
        bool is_endemic()       const { return (state & ENDEMIC) != 0;       }
        bool is_rare()          const { return (state & RARE) != 0;          }
        bool is_only_listened() const { return (state & ONLY_LISTENED) != 0; }
        void set_observed()           { state |= OBSERVED;                   }
        void set_unobserved()         { state &= ~OBSERVED;                  }
        void set_marked()             { state |= MARKED;                     }
        void set_unmarked()           { state &= ~MARKED;                    }
        void set_listened()           { state |= ONLY_LISTENED;              }
        void set_unlistened()         { state &= ~ONLY_LISTENED;             }
    };

    struct Genus {
        std::string chinese_name;
        std::string scientific_name;
        std::string english_name;
        QStandardItem* item;
        Family* family;
        std::map<std::string, Item*> items;
        int n_observed_items;
        int n_items;
    };

    struct Family {
        std::string chinese_name;
        std::string scientific_name;
        std::string english_name;
        cl::Array<Genus*> genus;
        Order* order;
        QStandardItem* item;
        int n_observed_items;
        int n_items;
    };

    struct Order {
        std::string chinese_name;
        std::string scientific_name;
        std::string english_name;
        cl::Array<Family*> famliers;
        QStandardItem* item;
        int n_observed_items;
        int n_items;
    };

    TaxonomicTree() {
        for (int i = 0; i < CONSERVATION_STRS.size(); ++i) {
            conservation_state_filter_.insert(i);
        }
    }

    /**
     * Count the number of obsered items.
     */
    void CountItems() {
        for (Order* order : orders_) {
            order->n_observed_items = 0;
            order->n_items = 0;
            for (Family* family : order->famliers) {
                family->n_items = 0;
                family->n_observed_items = 0;

                for (Genus* genus : family->genus) {
                    genus->n_items = 0;
                    genus->n_observed_items = 0;
                    for (const auto& k : genus->items) {
                        Item* item = k.second;
                        if (!InFilter(item->scientific_name)) continue;

                        if (item->is_observed()) {
                            ++genus->n_observed_items;
                            ++genus->n_items;
                        } else {
                            ++genus->n_items;
                        }
                    }
                    family->n_items += genus->n_items;
                    family->n_observed_items += genus->n_observed_items;
                }
                order->n_items += family->n_items;
                order->n_observed_items += family->n_observed_items;
            }
        }
    }

    /**
     * Load taxonomic tree from file.
     *
     * Return the error message.
     */
    QString LoadTree() {
        this->Clear();

        cl::io::LineReader line_reader;
        if (!line_reader.Open("taxonomic_tree.csv")) {
            return "Cannot open 'taxonomic_tree.csv'!";
        }

        Order* order = nullptr;
        Family* family = nullptr;

        while (char* line = line_reader.ReadLine()) {
            if (std::strlen(line) == 0) continue;

            cl::Array<std::string> terms;
            cl::StringSplit(line, ',', &terms);
            if (terms.size() != 4) {
                return "Error at line " + QString::number(line_reader.n_line());
            }

            InsertTranslation(terms[1], terms[3], &taxonomic_to_scientific_);
            InsertTranslation(terms[2], terms[3], &taxonomic_to_scientific_);
            InsertTranslation(terms[3], terms[3], &taxonomic_to_scientific_);

            if (terms[0] == "") {
                order = order_pool_.Allocate();
                order->chinese_name = terms[1];
                order->english_name = terms[2];
                order->scientific_name = terms[3];
                orders_.push_back(order);
                order_map_[order->scientific_name] = order;
            } else if (terms[0] == "----") {
                family = family_pool_.Allocate();
                family->chinese_name = terms[1];
                family->english_name = terms[2];
                family->scientific_name = terms[3];
                family->order = order;
                family_map_[family->scientific_name] = family;

                if (!order) return "Error at line " +
                                   QString::number(line_reader.n_line());

                order->famliers.push_back(family);
            } else if (terms[0] == "--------") {
                Genus* genus = genus_pool_.Allocate();
                genus->chinese_name = terms[1];
                genus->english_name = terms[2];
                genus->scientific_name = terms[3];
                genus->family = family;
                genus_map_[genus->scientific_name] = genus;

                if (!family) return "Error at line " +
                                    QString::number(line_reader.n_line());
                family->genus.push_back(genus);
            } else {
                return "Error at line " + QString::number(line_reader.n_line());
            }
        }
        return "";
    }

    /**
     * Load items from file.
     *
     * Return the error message.
     */
    QString LoadItems(const QString& account) {
        ClearItems();

        std::string str;

        cl::io::LineReader line_reader1;
        if (!line_reader1.Open("birds.csv")) {
            return "Cannot open 'birds.csv'!";
        }

        cl::io::LineReader line_reader2;
        if (!line_reader2.Open(account.toLocal8Bit().toStdString().c_str())) {
            return "Cannot open '" + account + "'!";
        }

        cl::io::LineReader line_reader3;
        if (!line_reader3.Open("iucn_list.csv")) {
            return "Cannot open 'iucn_list.csv'!";
        }

        std::string line;
        cl::Array<Item> items;

        std::map<std::string, int> hash;

        while (char* line = line_reader1.ReadLine()) {
            if (strlen(line) == 0) continue;

            cl::Array<std::string> terms;
            cl::StringSplit(line, ',', &terms);
            if (terms.size() < 4) {
                return "Error in 'birds.csv' at line: " +
                       QString::number(line_reader1.n_line());
            }

            Item item;
            item.chinese_name = terms[0];
            item.english_name = terms[1];
            item.scientific_name = terms[2];
            item.state = NONE;
            item.conservation_state = UNKNOWN;
            hash[terms[2]] = items.size();

            if (terms.size() == 5) {
                item.endemic_information = terms[4];
                if (!item.endemic_information.empty()) {
                    item.state |= ENDEMIC;
                }
            }

            cl::StringSplit(terms[3], ';', &item.information);
            while (!item.information.empty() && item.information.back().empty())
                item.information.pop_back();

            if (item.chinese_name.front() != '#')
                items.push_back(item);
        }

        while (char* line = line_reader2.ReadLine()) {
            if (strlen(line) == 0) continue;

            cl::Array<std::string> terms;
            cl::StringSplit(line, ',', &terms);

            if (terms.size() < 3) {
                return "Error in '" + account + "' at line: " +
                       QString::number(line_reader2.n_line());
            }

            if (hash.find(terms[0]) == hash.end()) continue;
            int id = hash[terms[0]];
            Item& item = items[id];
            item.state |= std::atoi(terms[1].c_str());
            cl::StringSplit(terms[2], ';', &item.observation_locations);
            while (!item.observation_locations.empty() &&
                   item.observation_locations.back().empty())
                item.observation_locations.pop_back();
        }

        while (char* line = line_reader3.ReadLine()) {
            if (strlen(line) == 0) continue;

            cl::Array<std::string> terms;
            cl::StringSplit(line, ',', &terms);

            if (terms.size() != 2) {
                return "Error in 'iucn_list.csv' at line: " +
                       QString::number(line_reader3.n_line());
            }

            if (hash.find(terms[0]) == hash.end()) {
                continue;
            }
            int id = hash[terms[0]];
            Item& item = items[id];

            for (int i = 0; i < CONSERVATION_STRS.size(); ++i) {
                if (terms[1] == CONSERVATION_STRS[i]) {
                    item.conservation_state = static_cast<ConservationState>(i);
                    break;
                }
            }
        }

        filter_.clear();
        for (const Item& item : items) {
            InsertItem(item);
        }
        fixed_filter_ = filter_;
        return "";
    }

    /**
     * Add a new species and immediately save the "bird.csv"
     */
    void AddSpecies(const Item& item) {
        InsertItem(item);
    }

    /**
     * Remove a species and immediately save the "bird.csv"
     */
    void RemoveSpecies(const Item& item) {
        Genus* genus = GetGenus(item.scientific_name);
        genus->items.erase(item.scientific_name);
    }

    /**
     * Setup taxonomic tree model.
     *
     * Please release the QT items before.
     *
     * Return the QStandardItemModel.
     */
    QStandardItemModel* SetupModel() {
        this->CountItems();

        ReleaseUIItems();
        QStandardItemModel* model = new QStandardItemModel();
        model->setHorizontalHeaderLabels(QStringList() << "Taxonomic Tree");

        int n_orders = 0;
        for (Order* order : orders_) {
            if (order->n_items == 0) continue;
            QString name = QString::fromStdString(order->chinese_name + " (" +
                                                  order->scientific_name + ")");
            order->item = new QStandardItem(name);
            order->item->setEditable(false);
            model->setItem(n_orders++, order->item);
            for (Family* family  : order->famliers) {
                if (family->n_items == 0) continue;

                QString name =
                        QString::fromStdString(family->chinese_name + " (" +
                                               family->scientific_name + ")");
                family->item = new QStandardItem(name);
                family->item->setEditable(false);
                order->item->appendRow(family->item);

                for (Genus* genus : family->genus) {
                    if (genus->n_items == 0) continue;

                    QString name =
                            QString::fromStdString(genus->chinese_name + " (" +
                                                   genus->scientific_name +
                                                   ")");
                    genus->item = new QStandardItem(name);
                    genus->item->setEditable(false);
                    family->item->appendRow(genus->item);
                }
            }
        }

        return model;
    }

    /**
     * Return the item of the given (english/scientific/chinese) name.
     */
    Item* GetItem(const std::string& name) const {
        const std::string scientific_name = GetItemName(name);
        auto i = item_map_.find(scientific_name);
        if (i == item_map_.end()) return nullptr;
        return item_map_.find(scientific_name)->second;
    }

    /**
     * Return the item of the given scientific name.
     */
    Item* get_item(const std::string& scientific_name) {
        return item_map_[scientific_name];
    }

    /**
     * Get genus items or family items.
     */
    void GetItems(const std::string& name, cl::Array<Item*>* items) const {
        assert(items);

        items->clear();
        const std::string scientific_name = GetTaxonomicName(name);

        auto i = family_map_.find(scientific_name);
        if (i != family_map_.end()) {
            Family* family = i->second;
            for (Genus* genus : family->genus) {
                for (auto& k : genus->items) {
                    // Remove the extra state.
                    k.second->state &= (MARKED | OBSERVED | ONLY_LISTENED |
                                        ENDEMIC);

                    if (InFilter(k.second->scientific_name)) {
                        // If in the filter, add the extra state.
                        k.second->state |= fixed_filter_.find
                                (k.second->scientific_name)->second;
                        items->push_back(k.second);
                    }
                }
            }
        }

        auto j = genus_map_.find(scientific_name);
        if (j != genus_map_.end()) {
            Genus* genus = j->second;
            for (auto& k : genus->items) {
                // Remove the extra state.
                k.second->state &= (MARKED | OBSERVED | ONLY_LISTENED |
                                    ENDEMIC);

                if (InFilter(k.second->scientific_name)) {
                    // If in the filter, add the extra state.
                    k.second->state |=
                        fixed_filter_.find(k.second->scientific_name)->second;
                    items->push_back(k.second);
                }
            }
        }
    }

    /**
     * Save items to the file.
     */
    void SaveItems(const std::string& account) const {
        std::ofstream fout("birds.csv");
        for (Order* order : orders_) {
            for (Family* family : order->famliers) {
                for (Genus* genus : family->genus) {
                    for (const auto& p3 : genus->items) {
                        Item* item = p3.second;
                        std::string information;
                        for (auto& t : item->information) {
                            information += t + ";";
                        }
                        for (size_t i = 0; i < information.size(); ++i) {
                            if (information[i] == ',') {
                                information[i] = ';';
                            }
                        }

                        fout << item->chinese_name << ","
                             << item->english_name << ","
                             << item->scientific_name << ","
                             << information;
                        if (item->endemic_information.empty()) {
                            fout << std::endl;
                        } else {
                            fout << "," << item->endemic_information
                                 << std::endl;
                        }
                    }
                }
            }
        }
        fout.close();

        std::ofstream fout1(account);
        for (Order* order : orders_) {
            for (Family* family : order->famliers) {
                for (Genus* genus : family->genus) {
                    for (const auto& p3 : genus->items) {
                        Item* item = p3.second;
                        std::string locations;
                        for (auto& t : item->observation_locations) {
                            locations += t + ";";
                        }
                        for (size_t i = 0; i < locations.size(); ++i) {
                            if (locations[i] == ',') {
                                locations[i] = ';';
                            }
                        }

                        fout1 << item->scientific_name << ","
                              << (item->state & ~TaxonomicTree::ENDEMIC) << ","
                              << locations
                              << std::endl;
                    }
                }
            }
        }
        fout1.close();

        SaveIUCN();
    }

    /**
     * Save IUCN list to the file.
     */
    void SaveIUCN() const {
        std::ofstream fout("iucn_list.csv");
        for (Order* order : orders_) {
            for (Family* family : order->famliers) {
                for (Genus* genus : family->genus) {
                    for (const auto& p3 : genus->items) {
                        Item* item = p3.second;

                        fout << item->scientific_name << ","
                             << CONSERVATION_STRS[item->conservation_state]
                             << std::endl;
                    }
                }
            }
        }
        fout.close();
    }

    QStandardItem* Search(const std::string& name) {
        std::string str = cl::StringTrim(name.c_str());
        str = UppercaseToLowercase(str);

        std::string scientific_name = GetItemName(str);
        if (!scientific_name.empty()) {
            auto item = item_map_.find(scientific_name);
            if (item != item_map_.end()) {
                return item->second->genus->item;
            }
            return nullptr;
        }

        scientific_name = GetTaxonomicName(str);
        if (!scientific_name.empty()) {
            auto genus = genus_map_.find(scientific_name);
            if (genus != genus_map_.end()) {
                return genus->second->item;
            }

            auto family = family_map_.find(scientific_name);
            if (family != family_map_.end()) {
                return family->second->item;
            }

            auto order = order_map_.find(scientific_name);
            if (order != order_map_.end()) {
                return order->second->item;
            }
        }
        return nullptr;
    }

    std::string GetTaxonomicName(const std::string& name) const {
        auto i = taxonomic_to_scientific_.find(UppercaseToLowercase(name));
        if (i != taxonomic_to_scientific_.end()) return i->second;

        return ""; // Not found.
    }

    std::string GetItemName(const std::string& name) const {
        auto i = item_to_scientific_.find(
              UppercaseToLowercase(cl::StringTrim(name)));
        if (i != item_to_scientific_.end()) return i->second;

        return ""; // Not found.
    }

    /**
     * Load the filter. Return items that not found.
     */
    void LoadFilter(const std::string& filter) {
        filter_.clear();

        if (filter != "All") {
            std::string file = filter;
            std::string str;
            cl::io::LineReader line_reader;
            if (!line_reader.Open(file)) return;

            while (char* line = line_reader.ReadLine()) {
                if (strlen(line) == 0) continue;
                if (line[0] == '#') continue;

                cl::Array<std::string> terms;
                cl::StringSplit(line, '\t', &terms);
                if (terms.empty()) continue;

                std::string found_name;
                Item* item = nullptr;
                for (const std::string& term : terms) {
                    std::string name = GetItemName(term);
                    item = GetItem(name);
                    if (item) {
                        found_name = name;
                        if (item->is_endemic()) {
                            filter_[name] = ENDEMIC;
                        } else {
                            filter_[name] = 0;
                        }
                        break;
                    }
                }

                if (!item) {
                    LOG(INFO) << "Not found bird: " << line;
                } else {
                    for (const std::string& term : terms) {
                        if (term.find("Rare") != term.npos) {
                            filter_[found_name] = RARE;
                            break;
                        }

                        if (term.find("Endemic") != term.npos) {
                            filter_[found_name] = ENDEMIC;
                            break;
                        }
                    }
                }
            }
        } else {
            for (Order* order : orders_) {
                for (Family* family : order->famliers) {
                    for (Genus* genus : family->genus) {
                        for (const auto& p3 : genus->items) {
                            Item* item = p3.second;
                            if (item->is_endemic()) {
                                filter_[item->scientific_name] = ENDEMIC;
                            } else {
                                filter_[item->scientific_name] = 0;
                            }
                        }
                    }
                }
            }
        }

        FixupFilter();
    }

    /**
     * Save lifer list of the current filter.
     *
     * The data is saved in the file "lifer_list.csv".
     */
    void SaveLiferList() const {
        std::ofstream fout("lifer_list.csv");

        for (auto a : fixed_filter_) {
            Item* item = GetItem(a.first);
            if (!item->is_observed()) {
                fout << item->chinese_name << "\t"
                     << item->english_name << "\t"
                     << item->scientific_name << std::endl;
            }
        }
    }

    /**
     * Save observed list of the current filter.
     *
     * The data is saved in the file "observed_list.csv".
     */
    void SaveObservedList() const {
        std::ofstream fout("observed_list.csv");

        for (auto a : fixed_filter_) {
            Item* item = GetItem(a.first);
            if (item->is_observed()) {
                fout << item->chinese_name << "\t"
                     << item->english_name << "\t"
                     << item->scientific_name << std::endl;
            }
        }
    }

    void ModifyScientificName(const std::string& scientific_name1,
                              const std::string& scientific_name2) {
        auto i = item_map_.find(scientific_name1);
        assert(i != item_map_.end());

        Item* item = i->second;
        item_map_.erase(scientific_name1);
        item->scientific_name = scientific_name2;
        item_map_[scientific_name2] = item;
    }

    bool InFilter(const Item* item) const {
        return InFilter(item->scientific_name);
    }

    bool InFilter(const std::string& name) const {
        return fixed_filter_.find(name) != fixed_filter_.end();
    }

    /**
     * Retrun true if the given conservation state is in the filter.
     */
    bool InFilter(ConservationState state) const {
        return conservation_state_filter_.find(state) !=
               conservation_state_filter_.end();
    }

    /**
     * Fixup the filter according to the options.
     *
     * Call it when options changed.
     */
    void FixupFilter() {
        fixed_filter_.clear();
        for (auto a : filter_) {
            Item* item = GetItem(a.first);
            CHECK(item);

            if (only_endemic_birds_ && a.second != ENDEMIC) continue;
            if (only_non_rare_birds_ && (item->conservation_state == EXTINCT ||
                                         a.second == RARE)) continue;
            if (!InFilter(item->conservation_state)) continue;
            auto difficulty_band_it = difficulty_band_map_.find(a.first);
            int difficulty_band = (difficulty_band_it == difficulty_band_map_.end())
                                  ? -1 : difficulty_band_it->second;
            if (!InDifficultyFilter(difficulty_band)) continue;
            if (only_unobserved_birds_ && item->is_observed()) continue;

            fixed_filter_[a.first] = a.second;
        }
    }

    void set_only_endemic_birds(bool flag) {
        only_endemic_birds_ = flag;
    }

    void set_only_non_rare_birds(bool flag) {
        only_non_rare_birds_ = flag;
    }

    void set_only_unobserved_birds(bool flag) {
        only_unobserved_birds_ = flag;
    }

    /**
     * Show or hide specfic conservation state.
     */
    void show_conservation_state(ConservationState state, bool flag) {
        if (flag)
            conservation_state_filter_.insert(state);
        else
            conservation_state_filter_.erase(state);
    }

    /**
     * Record the difficulty band (0..5) of a species, keyed by scientific name.
     */
    void set_difficulty_band(const std::string& scientific_name, int band) {
        difficulty_band_map_[scientific_name] = band;
    }

    /**
     * Return true if the given difficulty band (0..5) is in the filter.
     * Species without a recorded band (band < 0) are never filtered here.
     */
    bool InDifficultyFilter(int band) const {
        return band < 0 ||
               difficulty_filter_.find(band) != difficulty_filter_.end();
    }

    /**
     * Show or hide a specific difficulty band.
     */
    void show_difficulty_band(int band, bool flag) {
        if (flag)
            difficulty_filter_.insert(band);
        else
            difficulty_filter_.erase(band);
    }

    const cl::Array<Order*>& orders() const {
        return orders_;
    }

    const std::map<std::string, std::string>& item_to_scientific() const {
        return item_to_scientific_;
    }

    const std::map<std::string, std::string>& taxonomic_to_scientific() const {
        return taxonomic_to_scientific_;
    }

private:
    /**
     * Clear the data.
     */
    void Clear() {
        ReleaseUIItems();

        orders_.clear();
        order_pool_.clear();
        family_pool_.clear();
        genus_pool_.clear();
        item_pool_.clear();

        order_map_.clear();
        family_map_.clear();
        genus_map_.clear();
        item_map_.clear();

        filter_.clear();
        fixed_filter_.clear();
        item_to_scientific_.clear();
        taxonomic_to_scientific_.clear();
    }

    /**
     * Insert an item.
     */
    void InsertItem(const Item& item) {        
        Genus* genus = GetGenus(item.scientific_name);

        Item* new_item = item_pool_.Allocate();
        *new_item = item;
        new_item->genus  = genus;
        new_item->family = genus->family;
        new_item->order  = genus->family->order;

        genus->items[item.scientific_name] = new_item;
        assert(!item.scientific_name.empty());
        item_map_[item.scientific_name] = new_item;

        InsertTranslation(item.english_name, item.scientific_name,
                          &item_to_scientific_);
        InsertTranslation(item.chinese_name, item.scientific_name,
                          &item_to_scientific_);
        InsertTranslation(item.scientific_name, item.scientific_name,
                          &item_to_scientific_);
        if (item.is_endemic()) {
            filter_[item.scientific_name] = ENDEMIC;
        } else {
            filter_[item.scientific_name] = 0;
        }
    }

    /**
     * Clear all items.
     */
    void ClearItems() {
        item_pool_.clear();
        item_map_.clear();
        item_to_scientific_.clear();

        for (Order* order : orders_) {
            for (Family* family : order->famliers) {
                for (Genus* genus : family->genus) {
                    genus->items.clear();
                }
            }
        }
    }

    /**
     * Release UI items.
     *
     * Note that, we simply set the pointer to null. The memory data is released
     * in main_window.cc.
     */
    void ReleaseUIItems() {
        for (Order* order : orders_) {
            order->item = nullptr;
            for (Family* family : order->famliers) {
                family->item = nullptr;
                for (Genus* genus : family->genus) {
                    genus->item = nullptr;
                }
            }
        }
    }

    void InsertTranslation(const std::string& a, const std::string& b,
                           std::map<std::string, std::string>* map) {
        if (!a.empty() && !b.empty()) {
            (*map)[a] = b;
            (*map)[UppercaseToLowercase(a)] = b;
        }
    }

    static std::string UppercaseToLowercase(const std::string& str) {
        std::string res = str;
        for (size_t i = 0; i < res.size(); ++i) {
            if (res[i] >= 'A' && res[i] <= 'Z') {
                res[i] = res[i] - 'A' + 'a';
            }
        }
        return res;
    }

    /**
     * Get the genus of the given scientific name.
     */
    Genus* GetGenus(const std::string& scientific_name) const {
        cl::Array<std::string> terms;
        cl::StringSplit(scientific_name, &terms);
        assert(!terms.empty());
        assert(genus_map_.find(terms[0]) != genus_map_.end());

        return genus_map_.at(terms[0]);
    }

    // Original filter.
    std::map<std::string, int> filter_;
    // Fixed filter.
    std::map<std::string, int> fixed_filter_;
    // Conservation state filter.
    std::set<int> conservation_state_filter_;
    // Difficulty band (0..5) of each species, keyed by scientific name.
    std::map<std::string, int> difficulty_band_map_;
    // Difficulty bands (0..5) currently shown. All bands on by default.
    std::set<int> difficulty_filter_ = {0, 1, 2, 3, 4, 5};

    // Orders.
    cl::Array<Order*> orders_;

    // For tanslation.
    std::map<std::string, std::string> item_to_scientific_;
    std::map<std::string, std::string> taxonomic_to_scientific_;

    // To build tree.
    std::map<std::string, Order*> order_map_;
    std::map<std::string, Family*> family_map_;
    std::map<std::string, Genus*> genus_map_;
    std::map<std::string, Item*> item_map_;
    cl::Pool<Item> item_pool_;
    cl::Pool<Genus> genus_pool_;
    cl::Pool<Family> family_pool_;
    cl::Pool<Order> order_pool_;

    // Options.
    bool only_endemic_birds_  = false;
    bool only_non_rare_birds_ = false;
    bool only_unobserved_birds_ = false;
};

#endif // TAXONOMIC_TREE_H_
