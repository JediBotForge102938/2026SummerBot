(function () {
  const storageKey = "stryker-ftc-theme";
  const savedTheme = window.localStorage.getItem(storageKey);
  const theme = savedTheme === "light" ? "light" : "dark";
  document.documentElement.dataset.theme = theme;

  const picker = document.createElement("label");
  picker.className = "theme-picker";
  picker.innerHTML = '<span>Theme</span><select aria-label="Theme"><option value="dark">Dark</option><option value="light">Light</option></select>';

  const select = picker.querySelector("select");
  select.value = theme;
  select.addEventListener("change", function () {
    document.documentElement.dataset.theme = select.value;
    window.localStorage.setItem(storageKey, select.value);
  });

  const navigation = document.querySelector(".site-nav");
  if (navigation) {
    navigation.appendChild(picker);
  }
}());
