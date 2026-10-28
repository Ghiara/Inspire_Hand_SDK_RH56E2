"use strict";

(() => {
  const input = document.querySelector("#api-search");
  const status = document.querySelector("#search-status");
  const empty = document.querySelector("#no-results");
  const sections = [...document.querySelectorAll(".api-section")];
  const entries = [...document.querySelectorAll(".api-entry")];
  const navigation = [...document.querySelectorAll("nav a")];
  const text = new Map(entries.map(entry => [entry, entry.textContent.toLocaleLowerCase()]));

  document.querySelector(".search-box").hidden = false;

  function filter() {
    const terms = input.value.trim().toLocaleLowerCase().split(/\s+/).filter(Boolean);
    let visible = 0;
    for (const section of sections) {
      // Searching a controller name also reveals all of its methods.
      const title = section.querySelector("h2").textContent.toLocaleLowerCase();
      let count = 0;
      for (const entry of section.querySelectorAll(".api-entry")) {
        const match = terms.every(term => `${title} ${text.get(entry)}`.includes(term));
        entry.hidden = !match;
        if (match) count++;
      }
      section.hidden = count === 0;
      visible += count;
    }
    status.textContent = terms.length ? `找到 ${visible} 项，共 ${entries.length} 项` : `${entries.length} 项接口与说明`;
    empty.hidden = visible !== 0;
  }

  input.addEventListener("input", filter);
  input.addEventListener("keydown", event => {
    if (event.key === "Escape") {
      input.value = "";
      filter();
    }
  });

  function revealAnchor() {
    let id;
    try { id = decodeURIComponent(location.hash.slice(1)); }
    catch { return; }
    const target = document.getElementById(id);
    if (!target || !target.closest(".api-section")) return;
    // Permalinks remain usable even if a previous search hid the destination.
    if (input.value) {
      input.value = "";
      filter();
    }
    target.scrollIntoView({ block: "start" });
  }

  for (const anchor of document.querySelectorAll('a[href^="#"]')) {
    anchor.addEventListener("click", () => {
      if (input.value) {
        input.value = "";
        filter();
      }
    });
  }
  window.addEventListener("hashchange", revealAnchor);

  if ("IntersectionObserver" in window) {
    const observer = new IntersectionObserver(records => {
      for (const record of records) {
        if (!record.isIntersecting) continue;
        for (const anchor of navigation) {
          if (anchor.hash === `#${record.target.id}`) anchor.setAttribute("aria-current", "location");
          else anchor.removeAttribute("aria-current");
        }
      }
    }, { rootMargin: "-80px 0px -60% 0px", threshold: 0 });
    sections.forEach(section => observer.observe(section));
  }

  filter();
  if (location.hash) requestAnimationFrame(revealAnchor);
})();
