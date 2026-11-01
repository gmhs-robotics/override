//! Touch-only route menu using the original ozton selector theme and row layout.
use ozton::record::{routes::RouteIndex, selector::THEME_DARK};
use vexide::{
    color::Color,
    display::{Display, Font, FontFamily, FontSize, Line, Rect, Text, TouchState},
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Action {
    Disabled,
    Score,
    RecordNew,
    Playback(u32),
    Replace(u32),
    RetrySave,
}

#[derive(Clone)]
struct Choice {
    label: String,
    action: Action,
}

pub struct Selection {
    choices: Vec<Choice>,
    selected: usize,
    page: usize,
}

impl Selection {
    pub fn new(index: &RouteIndex, enabled: bool, pending: bool) -> Self {
        let mut choices = vec![
            Choice {
                label: "Disable".into(),
                action: Action::Disabled,
            },
            Choice {
                label: "Raise / Score / Retreat".into(),
                action: Action::Score,
            },
            Choice {
                label: "Record New Route".into(),
                action: Action::RecordNew,
            },
        ];
        if pending {
            choices.push(Choice {
                label: "Retry Save".into(),
                action: Action::RetrySave,
            });
        }
        for (id, name) in index.iter() {
            choices.push(Choice {
                label: format!("Play: {name}"),
                action: Action::Playback(id),
            });
            choices.push(Choice {
                label: format!("Record over: {name}"),
                action: Action::Replace(id),
            });
        }
        Self {
            choices,
            selected: usize::from(enabled),
            page: 0,
        }
    }
    pub fn action(&self) -> Action {
        self.choices[self.selected].action
    }
    fn select_action(&mut self, action: Action) {
        self.selected = self
            .choices
            .iter()
            .position(|c| c.action == action)
            .unwrap_or(0);
        self.page = self.selected / 5;
    }
    /// Same five 36-pixel rows and bottom page navigation as the original selector.
    fn touch(&mut self, disabled: bool, x: i16, y: i16) -> bool {
        if !disabled || !(0..480).contains(&x) || !(0..216).contains(&y) {
            return false;
        }
        if y < 180 {
            let index = self.page * 5 + y as usize / 36;
            if index < self.choices.len() {
                self.selected = index;
                return true;
            }
        } else if self.choices.len() > 5 {
            let last_page = (self.choices.len() - 1) / 5;
            self.page = if x < 240 {
                self.page.saturating_sub(1)
            } else {
                (self.page + 1).min(last_page)
            };
            return true;
        }
        false
    }
}

pub struct RouteMenu {
    display: Display,
    selection: Selection,
    last_press: i32,
    active: bool,
    status: String,
}

impl RouteMenu {
    pub fn new(mut display: Display, index: &RouteIndex, enabled: bool) -> Self {
        display.set_render_mode(vexide::display::RenderMode::DoubleBuffered);
        let last_press = display.touch_status().press_count;
        let mut menu = Self {
            display,
            selection: Selection::new(index, enabled, false),
            last_press,
            active: false,
            status: String::new(),
        };
        menu.draw();
        menu
    }
    pub fn action(&self) -> Action {
        self.selection.action()
    }
    pub fn refresh(&mut self, index: &RouteIndex, pending: bool, selected: Action) {
        self.selection = Selection::new(index, false, pending);
        self.selection.select_action(selected);
        self.draw();
    }
    /// Sample every control tick; no controller buttons are used for selection.
    pub fn poll(&mut self, disabled: bool) -> Option<Action> {
        let touch = self.display.touch_status();
        let pressed = touch.press_count != self.last_press;
        self.last_press = touch.press_count;
        let active = disabled && touch.state != TouchState::Released;
        let mut action = None;
        if pressed && self.selection.touch(disabled, touch.point.x, touch.point.y) {
            self.status.clear();
            if touch.point.y < 180 {
                action = Some(self.selection.action());
                crate::diagnostics::event!("selection={:?}", self.selection.action());
            }
            self.active = active;
            self.draw();
        } else if active != self.active {
            self.active = active;
            self.draw();
        }
        action
    }
    pub fn status(&mut self, status: &str) {
        if self.status != status {
            crate::diagnostics::event!("status={}", status);
            self.status = status.to_owned();
            self.draw();
        }
    }
    fn short(text: &str, count: usize) -> String {
        let mut chars = text.chars();
        let mut label: String = chars.by_ref().take(count).collect();
        if chars.next().is_some() {
            label.push('…');
        }
        label
    }
    fn text(&mut self, text: &str, x: i16, y: i16, color: Color) {
        self.display.draw_text(
            &Text::from_string(
                text,
                Font::new(FontSize::MEDIUM, FontFamily::Proportional),
                [x, y],
            ),
            color,
            None,
        );
    }
    fn draw(&mut self) {
        let theme = THEME_DARK;
        self.display
            .fill(&Rect::new([0, 0], [480, 240]), theme.background_default);
        for row in 0..5 {
            let index = self.selection.page * 5 + row;
            let Some(choice) = self.selection.choices.get(index) else {
                continue;
            };
            let selected = index == self.selection.selected;
            let label = Self::short(&choice.label, 28);
            let background = if selected && self.active {
                theme.background_selected_active
            } else if selected {
                theme.background_selected
            } else {
                theme.background_default
            };
            self.display.fill(
                &Rect::from_dimensions([0, row as i16 * 36], 478, 34),
                background,
            );
            self.text(
                &label,
                8,
                row as i16 * 36 + 6,
                if selected {
                    theme.text_selected
                } else {
                    theme.text_default
                },
            );
            if row > 0 {
                self.display.fill(
                    &Line::new([0, row as i16 * 36 - 1], [480, row as i16 * 36 - 1]),
                    theme.border,
                );
            }
        }
        if self.selection.choices.len() > 5 {
            self.display
                .fill(&Line::new([0, 180], [480, 180]), theme.border);
            self.display
                .fill(&Line::new([240, 180], [240, 216]), theme.border);
            self.text("Prev", 8, 186, theme.text_default);
            self.text("Next", 248, 186, theme.text_default);
        }
        let status = if self.status.is_empty() {
            format!(
                "Selected: {}",
                self.selection.choices[self.selection.selected].label
            )
        } else {
            self.status.clone()
        };
        self.text(&Self::short(&status, 32), 8, 220, theme.text_default);
        self.display.render();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn touch_selects_disable_script_and_record_without_a_button() {
        let mut menu = Selection::new(&RouteIndex::default(), true, false);
        assert_eq!(menu.action(), Action::Score);
        assert!(menu.touch(true, 20, 10));
        assert_eq!(menu.action(), Action::Disabled);
        assert!(menu.touch(true, 20, 40));
        assert_eq!(menu.action(), Action::Score);
        assert!(menu.touch(true, 20, 80));
        assert_eq!(menu.action(), Action::RecordNew);
        assert!(!menu.touch(false, 20, 10));
        assert_eq!(menu.action(), Action::RecordNew);
        assert!(!menu.touch(true, -1, 80));
        assert!(!menu.touch(true, 480, 80));
    }
    #[test]
    fn saved_routes_replacement_and_paging_resolve_exact_ids() {
        let mut index = RouteIndex::default();
        for id in 1..=8 {
            index.update(id, &format!("Route {id}"));
        }
        let mut menu = Selection::new(&index, false, true);
        menu.select_action(Action::Playback(8));
        assert_eq!(menu.action(), Action::Playback(8));
        assert_eq!(menu.page, 3);
        menu.select_action(Action::RetrySave);
        assert_eq!(menu.action(), Action::RetrySave);
        menu.touch(true, 400, 190);
        assert_eq!(menu.page, 1);
        menu.touch(true, 40, 10);
        assert_eq!(menu.action(), Action::Replace(1));
        menu.touch(true, 20, 190);
        assert_eq!(menu.page, 0);
    }
}
